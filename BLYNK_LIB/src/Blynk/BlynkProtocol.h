/**
 * @file       BlynkProtocol.h
 * @author     Volodymyr Shymanskyy
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2015 Volodymyr Shymanskyy
 * @date       Jan 2015
 * @brief      Blynk protocol implementation
 *
 */

#ifndef BlynkProtocol_h
#define BlynkProtocol_h

#include <string.h>
#include <stdlib.h>
#include <Blynk/BlynkDebug.h>
#include <Blynk/BlynkProtocolDefs.h>
#include <Blynk/BlynkApi.h>
#include <utility/BlynkUtility.h>

// Optional transport hooks for clients which explicitly opt in.  Other
// platforms (notably ESP8266) compile the original v0.6.1 path unchanged.
#ifdef BLYNK_TRANSPORT_AWARE
#ifndef BLYNK_TRANSPORT_SERVICE
#define BLYNK_TRANSPORT_SERVICE() do {} while (0)
#endif
#ifndef BLYNK_TRANSPORT_TX_PENDING
#define BLYNK_TRANSPORT_TX_PENDING() false
#endif
#ifndef BLYNK_TRANSPORT_PENDING_TIMEOUT_MS
#define BLYNK_TRANSPORT_PENDING_TIMEOUT_MS (1000UL * BLYNK_HEARTBEAT)
#endif

// A transport may need a long protocol deadline while still probing the peer
// frequently. By default these preserve the stock Blynk behavior exactly.
// The ESP32 air-quality sketch overrides them locally; ESP8266 builds which do
// not define the macros continue to use BLYNK_HEARTBEAT/BLYNK_TIMEOUT_MS.
#ifndef BLYNK_PROTOCOL_PING_INTERVAL
#define BLYNK_PROTOCOL_PING_INTERVAL BLYNK_HEARTBEAT
#endif
#ifndef BLYNK_PROTOCOL_PING_RETRY_MS
#define BLYNK_PROTOCOL_PING_RETRY_MS BLYNK_TIMEOUT_MS
#endif
#endif

// Optional application-frame acknowledgement barrier. The defining sketch
// opts in explicitly; builds which do not define this macro (including the
// ESP8266 sensor) retain the original Blynk 0.6.1 wire and scheduling path.
// A bounded batch of BLYNK_CMD_HARDWARE frames is followed by a uniquely
// identified PING. TCP ordering makes the matching RESPONSE end-to-end proof
// that every preceding application frame reached and was parsed by the server.
// Field traces on the ESP32 path also proved that an exact 128-byte atomic
// write can make the otherwise healthy TCP stream one-way. If the combined
// application+PING frame lands on that boundary, append one additional valid
// PING and track its RESPONSE. This is protocol-safe padding and proves that
// every preceding byte crossed the ordered stream.
#ifdef BLYNK_APPLICATION_ACK_BARRIER
#ifndef BLYNK_TRANSPORT_AWARE
#error "BLYNK_APPLICATION_ACK_BARRIER requires BLYNK_TRANSPORT_AWARE"
#endif
#ifndef BLYNK_APPLICATION_ACK_AVOID_LENGTH
#define BLYNK_APPLICATION_ACK_AVOID_LENGTH 128
#endif
#ifndef BLYNK_APPLICATION_ACK_BATCH_SIZE
#define BLYNK_APPLICATION_ACK_BATCH_SIZE 4
#endif
#ifndef BLYNK_APPLICATION_ACK_PARTIAL_FLUSH_MS
#define BLYNK_APPLICATION_ACK_PARTIAL_FLUSH_MS 0
#endif
#endif

template <class Transp>
class BlynkProtocol
    : public BlynkApi< BlynkProtocol<Transp> >
{
    friend class BlynkApi< BlynkProtocol<Transp> >;
public:
    enum BlynkState {
        CONNECTING,
        CONNECTED,
        DISCONNECTED,
        TOKEN_INVALID,
    };

    BlynkProtocol(Transp& transp)
        : conn(transp)
        , authkey(NULL)
        , redir_serv(NULL)
        , lastActivityIn(0)
        , lastActivityOut(0)
        , lastHeartbeat(0)
        , msgIdOut(0)
        , msgIdOutOverride(0)
        , nesting(0)
#ifdef BLYNK_TRANSPORT_AWARE
        , transportTxWasPending(false)
        , transportTxPendingSince(0)
        , transportTxFlushedAt(0)
        , protocolPingPending_(false)
        , protocolPingId_(0)
        , protocolPingPendingSince_(0)
        , protocolPingSentCount_(0)
        , protocolPingVerifiedCount_(0)
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
        , applicationAckPending_(false)
        , applicationAckPingId_(0)
        , applicationAckPendingSince_(0)
        , applicationAckSentCount_(0)
        , applicationAckCheckpointCount_(0)
        , applicationAckVerifiedCount_(0)
        , applicationAckBoundaryAvoidedCount_(0)
        , applicationAckPartialCheckpointCount_(0)
        , applicationAckBatchProgress_(0)
        , applicationAckLastFrameAt_(0)
        , applicationAckForceNext_(false)
#endif
        , state(CONNECTING)
    {}

    bool connected() const { return state == CONNECTED; }

#ifdef BLYNK_TRANSPORT_AWARE
    millis_time_t lastInboundActivity() const { return lastActivityIn; }
    millis_time_t lastOutboundActivity() const { return lastActivityOut; }
    bool protocolPingPending() const { return protocolPingPending_; }
    uint16_t protocolPingId() const { return protocolPingId_; }
    millis_time_t protocolPingAge() const {
        return protocolPingPending_ ? BlynkMillis() - protocolPingPendingSince_ : 0;
    }
    uint32_t protocolPingSentCount() const { return protocolPingSentCount_; }
    uint32_t protocolPingVerifiedCount() const { return protocolPingVerifiedCount_; }
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
    bool applicationAckPending() const { return applicationAckPending_; }
    uint16_t applicationAckPingId() const { return applicationAckPingId_; }
    millis_time_t applicationAckAge() const {
        return applicationAckPending_ ? BlynkMillis() - applicationAckPendingSince_ : 0;
    }
    uint32_t applicationAckSentCount() const { return applicationAckSentCount_; }
    uint32_t applicationAckCheckpointCount() const { return applicationAckCheckpointCount_; }
    uint32_t applicationAckVerifiedCount() const { return applicationAckVerifiedCount_; }
    uint32_t applicationAckBoundaryAvoidedCount() const { return applicationAckBoundaryAvoidedCount_; }
    uint32_t applicationAckPartialCheckpointCount() const { return applicationAckPartialCheckpointCount_; }
    uint8_t applicationAckBatchProgress() const { return applicationAckBatchProgress_; }
    // Request that the next HARDWARE command and its checkpoint PING share one
    // atomic transport write.  This is used by recovery canaries so the proof
    // cannot be held as a separate tiny TCP write behind the frame it proves.
    void forceNextApplicationAck() { applicationAckForceNext_ = true; }
#endif

    bool isTokenInvalid() const { return state == TOKEN_INVALID; }

    bool connect(uint32_t timeout = BLYNK_TIMEOUT_MS*3) {
        conn.disconnect();
        state = CONNECTING;
        millis_time_t started = BlynkMillis();
        while ((state != CONNECTED) &&
               (BlynkMillis() - started < timeout))
        {
            run();
        }
        return state == CONNECTED;
    }

    void disconnect() {
        conn.disconnect();
#ifdef BLYNK_TRANSPORT_AWARE
        transportTxWasPending = false;
        transportTxPendingSince = 0;
        transportTxFlushedAt = 0;
        protocolPingPending_ = false;
        protocolPingId_ = 0;
        protocolPingPendingSince_ = 0;
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
        applicationAckPending_ = false;
        applicationAckPingId_ = 0;
        applicationAckPendingSince_ = 0;
        applicationAckBatchProgress_ = 0;
        applicationAckLastFrameAt_ = 0;
        applicationAckForceNext_ = false;
#endif
        state = DISCONNECTED;
        BLYNK_LOG1(BLYNK_F("Disconnected"));
    }

    bool run(bool avail = false);

    // TODO: Fixme
    void startSession() {
        conn.connect();
        state = CONNECTING;
        msgIdOut = 0;
#ifdef BLYNK_TRANSPORT_AWARE
        transportTxWasPending = false;
        transportTxPendingSince = 0;
        transportTxFlushedAt = 0;
        protocolPingPending_ = false;
        protocolPingId_ = 0;
        protocolPingPendingSince_ = 0;
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
        applicationAckPending_ = false;
        applicationAckPingId_ = 0;
        applicationAckPendingSince_ = 0;
        applicationAckBatchProgress_ = 0;
        applicationAckLastFrameAt_ = 0;
        applicationAckForceNext_ = false;
#endif
        lastHeartbeat = lastActivityIn = lastActivityOut = (BlynkMillis() - 5000UL);
    }

    void sendCmd(uint8_t cmd, uint16_t id = 0, const void* data = NULL, size_t length = 0, const void* data2 = NULL, size_t length2 = 0);

    void printBanner() {
#if defined(BLYNK_NO_FANCY_LOGO)
        BLYNK_LOG1(BLYNK_F("Blynk v" BLYNK_VERSION " on " BLYNK_INFO_DEVICE));
#else
        BLYNK_LOG1(BLYNK_F(BLYNK_NEWLINE
            "    ___  __          __" BLYNK_NEWLINE
            "   / _ )/ /_ _____  / /__" BLYNK_NEWLINE
            "  / _  / / // / _ \\/  '_/" BLYNK_NEWLINE
            " /____/_/\\_, /_//_/_/\\_\\" BLYNK_NEWLINE
            "        /___/ v" BLYNK_VERSION " on " BLYNK_INFO_DEVICE BLYNK_NEWLINE
        ));
#endif
    }

private:

#ifdef BLYNK_TRANSPORT_AWARE
    void sendTrackedProtocolPing();
#endif

    void internalReconnect() {
        state = CONNECTING;
        conn.disconnect();
#ifdef BLYNK_TRANSPORT_AWARE
        transportTxWasPending = false;
        transportTxPendingSince = 0;
        transportTxFlushedAt = 0;
        protocolPingPending_ = false;
        protocolPingId_ = 0;
        protocolPingPendingSince_ = 0;
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
        applicationAckPending_ = false;
        applicationAckPingId_ = 0;
        applicationAckPendingSince_ = 0;
        applicationAckBatchProgress_ = 0;
        applicationAckLastFrameAt_ = 0;
        applicationAckForceNext_ = false;
#endif
        BlynkOnDisconnected();
    }

    int readHeader(BlynkHeader& hdr);
    uint16_t getNextMsgId();

protected:
    void begin(const char* auth) {
        this->authkey = auth;
        lastHeartbeat = lastActivityIn = lastActivityOut = (BlynkMillis() - 5000UL);
#if !defined(BLYNK_NO_DEFAULT_BANNER)
        printBanner();
#endif
    }

    bool processInput(void);

    Transp& conn;

private:
    const char* authkey;
    char*       redir_serv;
    millis_time_t lastActivityIn;
    millis_time_t lastActivityOut;
    union {
        millis_time_t lastHeartbeat;
        millis_time_t lastLogin;
    };
    uint16_t msgIdOut;
    uint16_t msgIdOutOverride;
    uint8_t  nesting;
#ifdef BLYNK_TRANSPORT_AWARE
    bool transportTxWasPending;
    millis_time_t transportTxPendingSince;
    millis_time_t transportTxFlushedAt;
    bool protocolPingPending_;
    uint16_t protocolPingId_;
    millis_time_t protocolPingPendingSince_;
    uint32_t protocolPingSentCount_;
    uint32_t protocolPingVerifiedCount_;
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
    bool applicationAckPending_;
    uint16_t applicationAckPingId_;
    millis_time_t applicationAckPendingSince_;
    uint32_t applicationAckSentCount_;
    uint32_t applicationAckCheckpointCount_;
    uint32_t applicationAckVerifiedCount_;
    uint32_t applicationAckBoundaryAvoidedCount_;
    uint32_t applicationAckPartialCheckpointCount_;
    uint8_t applicationAckBatchProgress_;
    millis_time_t applicationAckLastFrameAt_;
    bool applicationAckForceNext_;
#endif
protected:
    BlynkState state;
};

#ifdef BLYNK_TRANSPORT_AWARE
template <class Transp>
void BlynkProtocol<Transp>::sendTrackedProtocolPing()
{
    if (protocolPingPending_ || state != CONNECTED || !conn.connected()) {
        return;
    }
    const uint16_t pingId = getNextMsgId();
    protocolPingPending_ = true;
    protocolPingId_ = pingId;
    protocolPingPendingSince_ = BlynkMillis();
    ++protocolPingSentCount_;
    sendCmd(BLYNK_CMD_PING, pingId);
}
#endif

template <class Transp>
bool BlynkProtocol<Transp>::run(bool avail)
{
    BLYNK_RUN_YIELD();
#ifdef BLYNK_TRANSPORT_AWARE
    BLYNK_TRANSPORT_SERVICE();
#endif

    if (state == DISCONNECTED) {
        return false;
    }

    // Detect nesting
    BlynkHelperAutoInc guard(nesting);
    if (msgIdOutOverride || nesting > 2) {
      //BLYNK_LOG1(BLYNK_F("Nested run() skipped"));
      return true;
    }

    if (conn.connected()) {
#ifdef BLYNK_TRANSPORT_AWARE
        while (avail || conn.available() >= (int)sizeof(BlynkHeader)) {
#else
        while (avail || conn.available() > 0) {
#endif
            //BLYNK_LOG2(BLYNK_F("Available: "), conn.available());
            //const unsigned long t = micros();
            if (!processInput()) {
                conn.disconnect();
// TODO: Only when in direct mode?
#ifdef BLYNK_USE_DIRECT_CONNECT
                state = CONNECTING;
#endif
                BlynkOnDisconnected();
                return false;
            }
            avail = false;
            //BLYNK_LOG2(BLYNK_F("Proc time: "), micros() - t);
        }
    }

    const millis_time_t t = BlynkMillis();
#ifdef BLYNK_TRANSPORT_AWARE
    const bool txPending = BLYNK_TRANSPORT_TX_PENDING();

    if (txPending) {
        if (!transportTxWasPending) {
            transportTxWasPending = true;
            transportTxPendingSince = t;
        }
    } else if (transportTxWasPending) {
        // The ordered stream has only now been accepted by TCP. Send one
        // PING whose RESPONSE proves that all older bytes crossed the stream.
        transportTxWasPending = false;
        transportTxPendingSince = 0;
        transportTxFlushedAt = t;
        lastActivityOut = t;
        lastHeartbeat = t;
        if (state == CONNECTED && conn.connected()) {
            sendTrackedProtocolPing();
        }
    }
#endif

    // Update connection status after running commands
    const bool tconn = conn.connected();

    if (state == CONNECTED) {
        if (!tconn) {
            lastHeartbeat = t;
            internalReconnect();
            return false;
        }

#ifdef BLYNK_TRANSPORT_AWARE
        // Do not pile heartbeat frames behind bytes already waiting for TCP.
        // A bounded pending timeout still detects a path that cannot recover.
        if (txPending &&
            t - transportTxPendingSince > BLYNK_TRANSPORT_PENDING_TIMEOUT_MS) {
#ifdef BLYNK_DEBUG
            BLYNK_LOG2(BLYNK_F("Transport TX timeout: "), t - transportTxPendingSince);
#endif
            internalReconnect();
            return false;
        }

        const millis_time_t inboundAge = t - lastActivityIn;
        const millis_time_t flushAge = transportTxFlushedAt
            ? (t - transportTxFlushedAt) : inboundAge;
        const millis_time_t effectiveInboundAge =
            BlynkMin(inboundAge, flushAge);

#if defined(BLYNK_APPLICATION_ACK_BARRIER) && BLYNK_APPLICATION_ACK_PARTIAL_FLUSH_MS > 0
        // Do not leave the tail of a quiet application burst unverified. The
        // field failure ended at batch progress 3/4, so no checkpoint was
        // outstanding and the sketch could only use its conservative generic
        // inbound deadline. A delayed PING now closes that partial batch.
        if (!txPending && !applicationAckPending_ &&
            applicationAckBatchProgress_ > 0 &&
            t - applicationAckLastFrameAt_ >= BLYNK_APPLICATION_ACK_PARTIAL_FLUSH_MS) {
            const uint16_t checkpointId = getNextMsgId();
            applicationAckPending_ = true;
            applicationAckPingId_ = checkpointId;
            applicationAckPendingSince_ = t;
            applicationAckBatchProgress_ = 0;
            ++applicationAckCheckpointCount_;
            ++applicationAckPartialCheckpointCount_;
            lastHeartbeat = t;
            sendCmd(BLYNK_CMD_PING, checkpointId);
        }
#endif

        if (!txPending &&
            effectiveInboundAge > (1000UL * BLYNK_HEARTBEAT + BLYNK_TIMEOUT_MS*3)) {
#ifdef BLYNK_DEBUG
            BLYNK_LOG6(BLYNK_F("Heartbeat timeout: "), t, BLYNK_F(", "), lastActivityIn, BLYNK_F(", "), lastHeartbeat);
#else
            BLYNK_LOG1(BLYNK_F("Heartbeat timeout"));
#endif
            internalReconnect();
            return false;
        } else if (!txPending &&
#ifdef BLYNK_APPLICATION_ACK_BARRIER
                   !applicationAckPending_ &&
#endif
                   !protocolPingPending_ &&
                   (t - lastActivityIn  > 1000UL * BLYNK_PROTOCOL_PING_INTERVAL ||
                    t - lastActivityOut > 1000UL * BLYNK_PROTOCOL_PING_INTERVAL) &&
                    t - lastHeartbeat   > BLYNK_PROTOCOL_PING_RETRY_MS)
        {
            // Send ping if we didn't either send or receive something
            // for BLYNK_HEARTBEAT seconds
            sendTrackedProtocolPing();
            lastHeartbeat = t;
        }
#else
        if (t - lastActivityIn > (1000UL * BLYNK_HEARTBEAT + BLYNK_TIMEOUT_MS*3)) {
#ifdef BLYNK_DEBUG
            BLYNK_LOG6(BLYNK_F("Heartbeat timeout: "), t, BLYNK_F(", "), lastActivityIn, BLYNK_F(", "), lastHeartbeat);
#else
            BLYNK_LOG1(BLYNK_F("Heartbeat timeout"));
#endif
            internalReconnect();
            return false;
        } else if ((t - lastActivityIn  > 1000UL * BLYNK_HEARTBEAT ||
                    t - lastActivityOut > 1000UL * BLYNK_HEARTBEAT) &&
                    t - lastHeartbeat   > BLYNK_TIMEOUT_MS)
        {
            sendCmd(BLYNK_CMD_PING);
            lastHeartbeat = t;
        }
#endif
    } else if (state == CONNECTING) {
#ifdef BLYNK_USE_DIRECT_CONNECT
        if (!tconn)
            conn.connect();
#else
        if (tconn && (t - lastLogin > BLYNK_TIMEOUT_MS)) {
            BLYNK_LOG1(BLYNK_F("Login timeout"));
            conn.disconnect();
            state = CONNECTING;
            return false;
        } else if (!tconn && (t - lastLogin > 5000UL)) {
            conn.disconnect();
            if (!conn.connect()) {
                lastLogin = t;
                return false;
            }

            msgIdOut = 1;
            sendCmd(BLYNK_CMD_HW_LOGIN, 1, authkey, strlen(authkey));
            lastLogin = lastActivityOut;
            return true;
        }
#endif
    }
    return true;
}

template <class Transp>
BLYNK_FORCE_INLINE
bool BlynkProtocol<Transp>::processInput(void)
{
    BlynkHeader hdr;
    const int ret = readHeader(hdr);

    if (ret == 0) {
        return true; // Considered OK (no data on input)
    }

    if (ret < 0 || hdr.msg_id == 0) {
#ifdef BLYNK_DEBUG
        BLYNK_LOG2(BLYNK_F("Bad hdr len: "), ret);
#endif
        return false;
    }

    if (hdr.type == BLYNK_CMD_RESPONSE) {
        lastActivityIn = BlynkMillis();
#ifdef BLYNK_TRANSPORT_AWARE
        transportTxFlushedAt = 0;
        if (protocolPingPending_ &&
            hdr.msg_id == protocolPingId_ &&
            hdr.length == BLYNK_SUCCESS) {
            protocolPingPending_ = false;
            protocolPingId_ = 0;
            protocolPingPendingSince_ = 0;
            ++protocolPingVerifiedCount_;
        }
#endif
#ifdef BLYNK_APPLICATION_ACK_BARRIER
        if (applicationAckPending_ &&
            hdr.msg_id == applicationAckPingId_ &&
            hdr.length == BLYNK_SUCCESS) {
            applicationAckPending_ = false;
            applicationAckPingId_ = 0;
            applicationAckPendingSince_ = 0;
            ++applicationAckVerifiedCount_;
        }
#endif

#ifndef BLYNK_USE_DIRECT_CONNECT
        if (state == CONNECTING && (1 == hdr.msg_id)) {
            switch (hdr.length) {
            case BLYNK_SUCCESS:
            case BLYNK_ALREADY_REGISTERED:
                BLYNK_LOG3(BLYNK_F("Ready (ping: "), lastActivityIn-lastHeartbeat, BLYNK_F("ms)."));
                lastHeartbeat = lastActivityIn;
                state = CONNECTED;
#ifdef BLYNK_DEBUG
                if (size_t ram = BlynkFreeRam()) {
                    BLYNK_LOG2(BLYNK_F("Free RAM: "), ram);
                }
#endif
                this->sendInfo();
                BLYNK_RUN_YIELD();
                BlynkOnConnected();
                return true;
            case BLYNK_INVALID_TOKEN:
                BLYNK_LOG1(BLYNK_F("Invalid auth token"));
                state = TOKEN_INVALID;
                break;
            default:
                BLYNK_LOG2(BLYNK_F("Connect failed. code: "), hdr.length);
            }
            return false;
        }
        if (BLYNK_NOT_AUTHENTICATED == hdr.length) {
            return false;
        }
#endif
        // TODO: return code may indicate App presence
        return true;
    }

    if (hdr.length > BLYNK_MAX_READBYTES) {
        BLYNK_LOG2(BLYNK_F("Packet too big: "), hdr.length);
        // TODO: Flush
        internalReconnect();
        return true;
    }

    uint8_t inputBuffer[hdr.length+1]; // Add 1 to zero-terminate
    if (hdr.length != conn.read(inputBuffer, hdr.length)) {
#ifdef BLYNK_DEBUG
        BLYNK_LOG1(BLYNK_F("Can't read body"));
#endif
        return false;
    }
    inputBuffer[hdr.length] = '\0';

    BLYNK_DBG_DUMP(">", inputBuffer, hdr.length);

    lastActivityIn = BlynkMillis();
#ifdef BLYNK_TRANSPORT_AWARE
    transportTxFlushedAt = 0;
#endif

    switch (hdr.type)
    {
    case BLYNK_CMD_LOGIN:
    case BLYNK_CMD_HW_LOGIN: {
#ifdef BLYNK_USE_DIRECT_CONNECT
        if (strncmp(authkey, (char*)inputBuffer, 32)) {
            BLYNK_LOG1(BLYNK_F("Invalid token"));
            sendCmd(BLYNK_CMD_RESPONSE, hdr.msg_id, NULL, BLYNK_INVALID_TOKEN);
            break;
        }
#endif
        if (state == CONNECTING) {
            BLYNK_LOG1(BLYNK_F("Ready"));
            state = CONNECTED;
#ifdef BLYNK_DEBUG
            if (size_t ram = BlynkFreeRam()) {
                BLYNK_LOG2(BLYNK_F("Free RAM: "), ram);
            }
#endif
            this->sendInfo();
            BLYNK_RUN_YIELD();
            BlynkOnConnected();
        }
        sendCmd(BLYNK_CMD_RESPONSE, hdr.msg_id, NULL, BLYNK_SUCCESS);
    } break;
    case BLYNK_CMD_PING: {
        sendCmd(BLYNK_CMD_RESPONSE, hdr.msg_id, NULL, BLYNK_SUCCESS);
    } break;
    case BLYNK_CMD_REDIRECT: {
        if (!redir_serv) {
             redir_serv = (char*)malloc(32);
        }
        BlynkParam param(inputBuffer, hdr.length);
        uint16_t redir_port = BLYNK_DEFAULT_PORT; // TODO: Fixit

        BlynkParam::iterator it = param.begin();
        if (it >= param.end())
            return false;
        strncpy(redir_serv, it.asStr(), 32);
        if (++it < param.end())
            redir_port = it.asLong();
        BLYNK_LOG4(BLYNK_F("Redirecting to "), redir_serv, ':', redir_port);
        conn.disconnect();
        conn.begin(redir_serv, redir_port);
        state = CONNECTING;
        lastHeartbeat = lastActivityIn = lastActivityOut = (BlynkMillis() - 5000UL);
    } break;
    case BLYNK_CMD_HARDWARE:
    case BLYNK_CMD_BRIDGE: {
        msgIdOutOverride = hdr.msg_id;
        this->processCmd(inputBuffer, hdr.length);
        msgIdOutOverride = 0;
    } break;
    case BLYNK_CMD_INTERNAL: {
        BlynkReq req = { 0 };
        BlynkParam param(inputBuffer, hdr.length);
        BlynkParam::iterator it = param.begin();
        if (it >= param.end())
            return true;

        uint32_t cmd32;
        memcpy(&cmd32, it.asStr(), sizeof(cmd32));

        ++it;
        char* start = (char*)(it).asStr();
        unsigned length = hdr.length - (start - (char*)inputBuffer);
        BlynkParam param2(start, length);

        switch (cmd32) {
        case BLYNK_INT_RTC:  BlynkWidgetWriteInternalPinRTC(req, param2);    break;
        case BLYNK_INT_OTA:  BlynkWidgetWriteInternalPinOTA(req, param2);    break;
        case BLYNK_INT_ACON: BlynkWidgetWriteInternalPinACON(req, param2);   break;
        case BLYNK_INT_ADIS: BlynkWidgetWriteInternalPinADIS(req, param2);   break;
#ifdef BLYNK_DEBUG
        default:             BLYNK_LOG2(BLYNK_F("Invalid internal cmd:"), param.asStr());
#endif
        }
    } break;
    case BLYNK_CMD_DEBUG_PRINT: {
        if (hdr.length) {
            BLYNK_LOG2(BLYNK_F("Server: "), (char*)inputBuffer);
        }
    } break;
    default: {
#ifdef BLYNK_DEBUG
        BLYNK_LOG2(BLYNK_F("Invalid header type: "), hdr.type);
#endif
        // TODO: Flush
        internalReconnect();
    } break;
    }

    return true;
}

template <class Transp>
int BlynkProtocol<Transp>::readHeader(BlynkHeader& hdr)
{
#ifdef BLYNK_TRANSPORT_AWARE
    // Avoid entering blocking readBytes() for a fragmented 1..4 byte header.
    // The bytes remain untouched until the complete fixed header is present.
    if (conn.available() < (int)sizeof(hdr)) {
        return 0;
    }
#endif

    size_t rlen = conn.read(&hdr, sizeof(hdr));
    if (rlen == 0) {
        return 0;
    }

    if (sizeof(hdr) != rlen) {
        return -1;
    }

    BLYNK_DBG_DUMP(">", &hdr, sizeof(BlynkHeader));

    hdr.msg_id = ntohs(hdr.msg_id);
    hdr.length = ntohs(hdr.length);

    return rlen;
}

#ifndef BLYNK_SEND_THROTTLE
#define BLYNK_SEND_THROTTLE 0
#endif

#ifndef BLYNK_SEND_CHUNK
#define BLYNK_SEND_CHUNK 1024 // Just a big number
#endif

template <class Transp>
void BlynkProtocol<Transp>::sendCmd(uint8_t cmd, uint16_t id, const void* data, size_t length, const void* data2, size_t length2)
{
    if (!conn.connected() || (cmd != BLYNK_CMD_RESPONSE && cmd != BLYNK_CMD_PING && cmd != BLYNK_CMD_LOGIN && cmd != BLYNK_CMD_HW_LOGIN && state != CONNECTED) ) {
#ifdef BLYNK_DEBUG_ALL
        BLYNK_LOG2(BLYNK_F("Cmd skipped:"), cmd);
#endif
        return;
    }

#ifdef BLYNK_APPLICATION_ACK_BARRIER
    const bool applicationHardwareFrame = (cmd == BLYNK_CMD_HARDWARE);
    // Final protocol-side invariant. The sketch also checks this condition
    // before dequeuing, but no caller may put another application frame behind
    // an application frame which has not yet been verified end-to-end.
    if (applicationHardwareFrame && applicationAckPending_) {
        return;
    }
#endif

    if (0 == id) {
        id = getNextMsgId();
    }

#if defined(BLYNK_MSG_LIMIT) && BLYNK_MSG_LIMIT > 0
    if (cmd >= BLYNK_CMD_TWEET && cmd <= BLYNK_CMD_HARDWARE) {
        const millis_time_t allowed_time = BlynkMax(lastActivityOut, lastActivityIn) + 1000/BLYNK_MSG_LIMIT;
        int32_t wait_time = allowed_time - BlynkMillis();
        if (wait_time >= 0) {
#ifdef BLYNK_DEBUG_ALL
            BLYNK_LOG2(BLYNK_F("Waiting:"), wait_time);
#endif
            while (wait_time >= 0) {
                run();
                wait_time = allowed_time - BlynkMillis();
            }
        } else if (nesting == 0) {
            run();
        }
    }
#endif

#ifdef BLYNK_APPLICATION_ACK_BARRIER
    const size_t applicationFrameLength = (sizeof(BlynkHeader)) +
                                          (data  ? length  : 0) +
                                          (data2 ? length2 : 0);
    const bool applicationAckScheduled = applicationHardwareFrame &&
        (applicationAckForceNext_ ||
         applicationAckBatchProgress_ + 1U >= BLYNK_APPLICATION_ACK_BATCH_SIZE);
    // A bare 128-byte application write is also promoted to an ACK checkpoint,
    // so neither an unverified frame nor an application+PING write is ever
    // handed to the affected ESP32 transport at the field-proven boundary.
    const bool applicationAckBareBoundary = applicationHardwareFrame &&
        applicationFrameLength == BLYNK_APPLICATION_ACK_AVOID_LENGTH;
    const bool applicationAckThisFrame = applicationAckScheduled ||
                                         applicationAckBareBoundary;
    const uint16_t applicationAckFirstId = applicationAckThisFrame
        ? getNextMsgId() : 0;
    const size_t applicationAckLength = applicationAckThisFrame
        ? sizeof(BlynkHeader) : 0;
    const size_t applicationAckUnpaddedLength = applicationFrameLength +
                                                applicationAckLength;
    const bool applicationAckBoundaryPad = applicationAckThisFrame &&
        applicationAckUnpaddedLength == BLYNK_APPLICATION_ACK_AVOID_LENGTH;
    const uint16_t applicationAckId = applicationAckBoundaryPad
        ? getNextMsgId() : applicationAckFirstId;
    const size_t applicationAckBoundaryPadLength = applicationAckBoundaryPad
        ? sizeof(BlynkHeader) : 0;
#endif

#ifdef BLYNK_APPLICATION_ACK_BARRIER
    const size_t full_length = (sizeof(BlynkHeader)) +
                               (data  ? length  : 0) +
                               (data2 ? length2 : 0) +
                               applicationAckLength +
                               applicationAckBoundaryPadLength;
#else
    const size_t full_length = (sizeof(BlynkHeader)) +
                               (data  ? length  : 0) +
                               (data2 ? length2 : 0);
#endif

#if defined(BLYNK_SEND_ATOMIC) || defined(ESP8266) || defined(ESP32) || defined(SPARK) || defined(PARTICLE) || defined(ENERGIA)
    // Those have more RAM and like single write at a time...

    uint8_t buff[full_length];

    BlynkHeader* hdr = (BlynkHeader*)buff;
    hdr->type = cmd;
    hdr->msg_id = htons(id);
    hdr->length = htons(length+length2);

    size_t pos = sizeof(BlynkHeader);
    if (data && length) {
        memcpy(buff + pos, data, length);
        pos += length;
    }
    if (data2 && length2) {
        memcpy(buff + pos, data2, length2);
#ifdef BLYNK_APPLICATION_ACK_BARRIER
        pos += length2;
#endif
    }
#ifdef BLYNK_APPLICATION_ACK_BARRIER
    if (applicationAckThisFrame) {
        BlynkHeader ack;
        ack.type = BLYNK_CMD_PING;
        ack.msg_id = htons(applicationAckFirstId);
        ack.length = 0;
        memcpy(buff + pos, &ack, sizeof(ack));
        pos += sizeof(ack);
        if (applicationAckBoundaryPad) {
            ack.msg_id = htons(applicationAckId);
            memcpy(buff + pos, &ack, sizeof(ack));
        }
    }
#endif

    size_t wlen = 0;
    while (wlen < full_length) {
        const size_t chunk = BlynkMin(size_t(BLYNK_SEND_CHUNK), full_length - wlen);
        BLYNK_DBG_DUMP("<", buff + wlen, chunk);
        const size_t w = conn.write(buff + wlen, chunk);
        BlynkDelay(BLYNK_SEND_THROTTLE);
        if (w == 0) {
#ifdef BLYNK_DEBUG
            BLYNK_LOG1(BLYNK_F("Cmd error"));
#endif
            conn.disconnect();
            state = CONNECTING;
            BlynkOnDisconnected();
            return;
        }
        wlen += w;
    }

#else

    BlynkHeader hdr;
    hdr.type = cmd;
    hdr.msg_id = htons(id);
    hdr.length = htons(length+length2);

    BLYNK_DBG_DUMP("<", &hdr, sizeof(hdr));
    size_t wlen = conn.write(&hdr, sizeof(hdr));
    BlynkDelay(BLYNK_SEND_THROTTLE);

    if (cmd != BLYNK_CMD_RESPONSE) {
        if (length) {
            BLYNK_DBG_DUMP("<", data, length);
            wlen += conn.write(data, length);
            BlynkDelay(BLYNK_SEND_THROTTLE);
        }
        if (length2) {
            BLYNK_DBG_DUMP("<", data2, length2);
            wlen += conn.write(data2, length2);
            BlynkDelay(BLYNK_SEND_THROTTLE);
        }
    }

#endif

    if (wlen != full_length) {
#ifdef BLYNK_DEBUG
        BLYNK_LOG4(BLYNK_F("Sent "), wlen, '/', full_length);
#endif
        internalReconnect();
        return;
    }

    lastActivityOut = BlynkMillis();
#ifdef BLYNK_APPLICATION_ACK_BARRIER
    if (applicationHardwareFrame) {
        ++applicationAckSentCount_;
        applicationAckLastFrameAt_ = lastActivityOut;
        applicationAckForceNext_ = false;
        if (!applicationAckThisFrame) {
            ++applicationAckBatchProgress_;
        } else {
            applicationAckBatchProgress_ = 0;
            ++applicationAckCheckpointCount_;
            applicationAckPending_ = true;
            applicationAckPingId_ = applicationAckId;
            applicationAckPendingSince_ = lastActivityOut;
            if (applicationAckBoundaryPad || applicationAckBareBoundary) {
                ++applicationAckBoundaryAvoidedCount_;
            }
            lastHeartbeat = lastActivityOut;
        }
    }
#endif

}

template <class Transp>
uint16_t BlynkProtocol<Transp>::getNextMsgId()
{
    if (msgIdOutOverride != 0)
        return msgIdOutOverride;
    if (++msgIdOut == 0)
        msgIdOut = 1;
    return msgIdOut;
}

#endif

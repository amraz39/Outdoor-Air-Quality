// Forward declarations required because the Arduino build process may generate
// function prototypes before the library headers below are processed.
// - RtcCheckpoint: used by later RTC helpers.
// - arduino_event_t: used by the Core 3.x WiFi system-event callback.
// - BlynkReq/BlynkParam are provided by the Blynk headers; V55 is defined
//   directly as BlynkWidgetWrite55 to avoid Arduino auto-prototype linkage conflicts.
struct RtcCheckpoint;
struct arduino_event_t;
struct SensorData;  // Required before Arduino's auto-generated prototypes
struct Core1TelemetryPair;

// Controlled diagnostic switch:
//   0 = normal full firmware workload
//   1 = RX-isolation run: keep GPS/PPS, WiFi, Blynk and diagnostics alive while
//       bypassing the heavier sensor/application workload.
#ifndef ESP32_RX_ISOLATION_TEST
#define ESP32_RX_ISOLATION_TEST 0
#endif

// v3.9.213 ONE-SECOND OBSERVABLE LINK HEARTBEAT
//   - V4 remains a small random-looking 1..9 value, published once per second,
//     but consecutive values are forced to differ. Every successful
//     publication is therefore observable through the Blynk HTTP value API,
//     which exposes the current value but no per-write timestamp.
//   - V61/V62 remain the authoritative coherent sensor record and independent
//     fallback evidence. No sensor, HTTP-pair, or Blynk reconnect ownership is
//     moved or combined with the heartbeat.
//
// v3.9.212 RESET-CHURN REDUCTION EXPERIMENT
//   - A recent HTTP failure corroborates a Blynk sequence hole only while at
//     least two HTTP failures remain consecutive, or while the HTTP circuit
//     breaker is open. A recovered one-off HTTP timeout can no longer turn the
//     first Blynk hole into an immediate whole-MCU reset.
//   - First WiFi association waits three seconds after GOT_IP before opening
//     the Blynk socket. Re-association recovery remains immediate. This avoids
//     the repeatable first post-reset TCP timeout without delaying recovery.
//   - Proactively rotate the persistent HTTP telemetry socket after 60 requests
//     (previously 120), before the field-observed long-lived-PCB degradation.
//   - V55 is boot-locked for 30 seconds and edge-armed by a preceding LOW, so a
//     retained/replayed HIGH cannot cause another reset after reconnection.
//
// v3.9.211 CORROBORATED RESET-DOMAIN RECOVERY
//   - The v3.9.210 field trace proves why a Blynk-only reset often cannot
//     recover: the HTTP data plane timed out first, the established Blynk
//     stream then lost an exact PING, fresh SYNs timed out, and a separately
//     authenticated replacement session immediately became one-way. Only an
//     MCU reset cleared that shared WiFi/lwIP/security state.
//   - When an exact Blynk sequence hole is corroborated by a recent HTTP
//     transaction timeout or an active HTTP circuit breaker, bypass the known-
//     ineffective socket rotation and reset the proven shared state at once.
//   - A Blynk-only sequence hole with a healthy HTTP path still receives the
//     narrow socket-first recovery. HTTP failure alone never resets the MCU.
//   - No raw PCB mutation and no SO_LINGER dependency: Arduino-ESP32 3.3.12
//     ships its precompiled lwIP with CONFIG_LWIP_SO_LINGER disabled.
//
// v3.9.210 POST-RUN EXACT-PING HEALTH GATE
//   - Fixes v3.9.209 falsely isolating HTTP on every normal 10-second inbound
//     cadence before Blynk.run() consumed the response. That latched the 60s
//     holdoff forever and stopped V61/V62 after only 14 successful records.
//   - Evaluate after Blynk.run(); only an exact protocol PING still pending for
//     3s is a threat. Start the stable quiet timer once that threat clears.
// v3.9.209 CROSS-STREAM HEALTH ISOLATION
//   - A Blynk receive-path warning now closes the HTTP socket even when it is
//     idle, then holds bulk telemetry offline for a bounded quiet recovery
//     window. v3.9.208 paused new HTTP work but accidentally retained the
//     second persistent PCB, so rotating only Blynk could not clear a shared
//     lwIP/WiFi receive-path fault.
//   - A healthy HTTP socket is retired after a bounded number of requests.
//     This prevents one long-lived bulk PCB from accumulating the field-observed
//     500-request degradation while retaining low connection churn.
// v3.9.208 SINGLE-OWNER CORE-0 NETWORK PLANE
//   - Removes HTTPClient/WiFiClient and the telemetry network task from Core 1.
//     Core 1 now only formats immutable V61/V62 snapshots and overwrites the
//     one-slot queue; it never opens, writes, reads or closes a TCP socket.
//   - Core 0 owns Blynk and coherent HTTP transport. HTTP is a fixed-buffer,
//     O_NONBLOCK raw-lwIP state machine: connect/send/recv are polled with zero
//     wait and each loop pass performs at most one socket operation after Blynk.
//   - Retains persistent HTTP/1.1 reuse, latest-only 1s telemetry, failure
//     circuit breaking and direct whole-MCU recovery for a proven shared path.
//   - Adds owner/state/service-time/cancellation telemetry so field logs prove
//     that HTTP cannot stall Blynk or execute from the sensor core.
// v3.9.207 HTTP DEADLINE + CONNECTION-CHURN CIRCUIT BREAKER
//   - Field telemetry proves persistent reuse initially worked (1 connection,
//     43 reuses), then HTTP -11 read timeouts accumulated while Blynk remained
//     healthy. Core-1 sensor latency reaches 1.5s, beyond the former 1s HTTP
//     deadline; every false timeout closed a healthy socket and recreated churn.
//   - Drain the fixed two-byte OK body without a temporary String, use a 3s
//     response deadline, and run the HTTP worker above the sensor task priority.
//   - Three HTTP failures in 60s open a 90s bulk-channel circuit breaker so
//     FIN/TIME_WAIT state drains while Blynk remains fully serviced. Healthy
//     operation still sends the newest sealed pair every second.
//
// v3.9.206 PERSISTENT TELEMETRY SOCKET + PROVEN RESET DOMAIN
//   - The v3.9.205 trace failed after 272 successful HTTP records: HTTP
//     failed first, Blynk then became one-way, and every new TCP connect timed
//     out while WiFi remained associated. This is a shared lwIP-path failure.
//   - The former HTTPClient worker lived for the task lifetime and consumed each
//     two-byte OK body before reuse. Successful samples did not call end(), avoiding
//     per-record FIN/TIME_WAIT churn against the core's 16 active TCP PCBs.
//   - Socket generation, reuse and local port are reported to prove persistence.
//   - After a proven sequence hole plus two full TCP timeouts, restart the whole
//     MCU directly. The failed WiFi teardown/reassociation path is bypassed.
//
// v3.9.205 SINGLE-OWNER RECOVERY + 1-SECOND LATEST TELEMETRY
//   - The v3.9.204 field trace proved that scheduling shared-path recovery did
//     not suppress the remainder of loop(): another Blynk connect ran during
//     the quiesce window and overlapped the WiFi driver shutdown. Quiesce and
//     WAIT_GOT_IP now own the complete network portion of loop().
//   - The one authorized driver rebuild clears stale STA configuration and
//     forces an unpinned AP scan. If that clean association cannot obtain a
//     fresh GOT_IP in 30s, restart the MCU immediately instead of performing
//     further WiFi teardowns for another two minutes.
//   - V61/V62 return to one-second latest-value delivery. The separate HTTP
//     socket still pauses on Blynk staleness and retains exponential backoff.
//
// v3.9.204 COORDINATED SHARED-PATH RECOVERY
//   - When both independent TCP paths become unusable after a proven Blynk
//     sequence hole, quiesce both owners and rebuild only the ESP32 WiFi
//     driver/netif. Require a fresh GOT_IP before resuming Blynk, then delay
//     best-effort HTTP telemetry for a short post-IP grace period.
//   - Exact application-ACK and protocol-PING evidence now reacts at 25s;
//     generic silence remains conservative at 60s. Ordinary refusal/login
//     failures never authorize a WiFi rebuild.
//   - Treat a disconnect event without a newer GOT_IP as disconnected even if
//     Arduino-ESP32 temporarily leaves WiFi.status() at WL_CONNECTED.
//   - Rate-limit bulk telemetry to one latest record per 15s and pause it as
//     soon as the Blynk receive path becomes suspect.
//
// v3.9.203 ISOLATED BULK TELEMETRY DATA PLANE
//   - Packet captures proved that one missing ESP32->server TCP segment blocks
//     every later Blynk command behind the sequence hole. Timing tweaks and
//     socket/STA resets react to that fault but cannot prevent head-of-line
//     blocking on the same persistent stream.
//   - V61/V62 no longer enter the Blynk hardware socket. One Core-1 worker
//     sends the sealed pair to the server's dedicated coherent HTTP endpoint
//     over a separately owned persistent connection. Failure closes/retries
//     only that bulk channel; Blynk control, heartbeat and dashboard traffic
//     remain independent.
//   - The server endpoint stores both pins and forwards them to apps without
//     echoing the large strings back to the hardware connection. A bounded
//     latest-value queue prevents backlog and preserves the pair key.
//
// v3.9.202 SOCKET-FIRST RECOVERY; NO BLYNK-INITIATED STA REASSOCIATION
//   - The ESP-IDF 6.1 field run reproduced the same one-way TCP failure as
//     5.5.5, so changing the Espressif core did not remove the fault.
//   - More importantly, the first-hole soft STA reassociation did not repair
//     the path reliably: the hole recurred 63.5s later in one run, while the
//     next run entered NO_AP_FOUND, an association timeout, four full TCP
//     timeouts and one login timeout before returning after about 118s.
//   - A proven first hole now rotates only the owner-managed Blynk socket. A
//     separately authenticated replacement session which also becomes one-way,
//     or three full TCP connect timeouts immediately following that hole,
//     requests one controlled MCU restart. Field logs show that restart is the
//     only consistently successful broader reset domain.
//   - Blynk faults no longer disconnect or rebuild STA. Spontaneous real WiFi
//     events retain their independent bounded recovery state machine.
//   - This logic uses APIs common to Arduino-ESP32 3.3.12 and 4.0.0-RC1.
//
// v3.9.201 FIRST-HOLE SECURITY/STA REASSOCIATION
//   - The server-side capture is authoritative: later TCP bytes reached the Pi
//     while its ACK remained pinned at one missing ESP32 byte. Java/Blynk never
//     received a complete ordered command to process, so server tuning cannot
//     prevent this failure.
//   - A prior field run ultimately emitted WiFi reason 16 (group-key update
//     timeout). The first proven end-to-end sequence hole now rebuilds the STA
//     association immediately, renewing WPA keys plus AP forwarding, WiFi and
//     lwIP path state. v3.9.200 rotated only TCP first and left those lower
//     states unchanged.
//   - The reassociation is soft: no WIFI_OFF, credential erase, or immediate
//     MCU reset. If the same precise hole returns within five minutes after a
//     successful reassociation, one controlled MCU restart is the final bounded
//     fallback; field evidence shows reset restores this unit.
//   - Normal telemetry cadence, 802.11b/g/n, Blynk library and Java server are
//     unchanged. The previously tried no-11n/A-MPDU mode is not reintroduced.
//
// v3.9.200 TWO-HOUR RX-PATH + ASSOCIATION RECOVERY HARDENING
//   - Counts a failed authenticated replacement/canary socket as the second
//     independent RX-path generation and immediately leaves the poisoned STA
//     association instead of rotating a third socket on the same path.
//   - The first RX-path repair is now a clean STA reassociation. The v3.9.199
//     field trace proved that an immediate WIFI_OFF rebuild can provoke a
//     persistent reason=2 (AUTH_EXPIRE) association loop on this AP.
//   - WiFi association attempts receive 30 seconds. Timeouts use soft cancel
//     first, permit only one hard driver rebuild after three failed attempts,
//     and finally reboot the MCU after four minutes offline. This prevents the
//     former 20-second repeated WIFI_OFF loop while retaining a bounded last
//     resort for the observed condition that only reset clears.
//
// v3.9.199 FALSE-CONNECTED ESP32 RX-PATH RECOVERY
//   - The v3.9.198 field trace proves that the first socket-only repair can
//     authenticate, yet a second independent Blynk stream becomes one-way
//     about 83 seconds later while WL_CONNECTED, RSSI, heap and the WiFi event
//     sequence remain healthy. Fresh TCP connects then spend the complete
//     1000ms connect budget without reaching HW_LOGIN. This is below Blynk:
//     the associated ESP32 STA/lwIP receive path is no longer usable.
//   - Track full-duration TCP connect timeouts separately from immediate
//     connection refusals. Only timeouts following a proven sequence hole are
//     evidence for the false-connected receive-path state.
//   - The first isolated sequence hole still rotates only its Blynk socket. A
//     second independent hole within five minutes, or three real TCP connect
//     timeouts during that recovery generation, performs one bounded WiFi
//     driver/netif rebuild. Ordinary server/login failures never reset WiFi.
//   - The recovery closes the owner-managed Blynk socket, powers only the WiFi
//     driver off, waits for the existing non-blocking settle window, and starts
//     one clean association. Sensor acquisition and the MCU remain running.

// v3.9.91 STOCK BLYNK + COHERENT 1s SNAPSHOT + TX-CONTROLLED BUILD
//   - Makes the RX-isolation switch explicit near the top of the source.
//   - Adds a stall-time raw RX queue observation every 2s so firmware state can
//     be compared directly with the RPi tcpdump timeline.
//   - Repeated TCP-probe failures while WiFi is still genuinely healthy now
//     cause Blynk-only recovery instead of tearing down healthy WiFi. This
//     prevents the firmware from creating the later status=6/reason=2 cascade.
//   - WiFi teardown remains available when WiFi is actually offline or an
//     unexpected WiFi disconnect event has occurred.
//   - Existing sensor, GPS/PPS, INAV, Blynk, watchdog and diagnostics remain.

// v3.9.84 ESP32-SIDE RX ISOLATION + BLYNK-ONLY HANDSHAKE RECOVERY
//   - Treats a Blynk handshake failure as a Blynk/TCP problem while WiFi is
//     still genuinely healthy: WL_CONNECTED + no new unexpected WiFi event.
//     In that state the firmware now closes/resets only the Blynk transport
//     and leaves the WiFi association untouched. This prevents the first
//     Blynk RX failure from creating a secondary WiFi reason=2/status=6 loop.
//   - WiFi teardown remains available when WiFi is actually offline or an
//     unexpected WiFi disconnect occurred, so genuine WiFi failures still use
//     the existing recovery path.
//   - Adds an optional ESP32_RX_ISOLATION_TEST mode (default OFF) that keeps
//     GPS/PPS and the network diagnostics alive while bypassing the heavier
//     CO, ENS160/AHT2x, INMP441, BMI160/INAV and OLED application workloads.
//     This provides a controlled ESP32-side A/B test without deleting any
//     existing sensor logic. Set the macro to 1 for the isolation run.
//   - Adds per-stage sensor timing diagnostics so long ENS/I2C, microphone,
//     CO, GPS or INAV sections are identified instead of only reporting the
//     aggregate sensor cycle maximum.
//   - Keeps v3.9.83 raw five-byte Blynk RX header visibility and all existing
//     sensor, GPS, INAV, watchdog and diagnostic logic.
//
// v3.9.83 BLYNK RX HEADER CAPTURE VISIBLE + v3.9.81 RX ROOT-CAUSE DIAGNOSTICS
//   - Keeps the v3.9.82 raw 5-byte Blynk header decoder and adds the last
//     captured five bytes directly to every periodic RX trace, so the actual
//     packet content remains visible even if individual RX-header lines are
//     missed by the serial monitor/filter.
//   - Adds a monotonically increasing RX-header sequence number and a
//     last-header timestamp/hex field. No socket behavior or recovery logic
//     is changed.
//   - Adds passive Blynk stream framing diagnostics to capture the actual
//     5-byte header of every newly received Blynk message without consuming or
//     changing the TCP stream. The header is decoded as command, message ID,
//     and body length so the recurring 5-byte RX packets can be identified.
//   - Tracks PING (cmd=6), RESPONSE (cmd=0), HARDWARE (cmd=20), INTERNAL
//     (cmd=17), and other received command counts plus the last decoded header.
//   - Existing sensor, GPS, INAV, Blynk, watchdog and recovery logic is kept.
//
// v3.9.81 RX ROOT-CAUSE DIAGNOSTICS + WDT HARDENING
//   - Adds independent TCP RX accounting below the Blynk protocol layer:
//     recv() calls, data/EOF/error returns, EAGAIN/EINTR counts, total RX bytes,
//     last RX time, largest read, and per-trace deltas.
//   - Adds a periodic connected-session RX trace so the healthy->stalled
//     transition is captured, not only the already-stalled 10s/25s state.
//   - Adds passive MSG_PEEK/connected() result counters and correlates them with
//     Blynk.run(), TX, heartbeat, socket and WiFi state.
//   - Keeps v3.9.80 Blynk-only RX recovery and hard WiFi teardown unchanged.
//   - Hardens the task-WDT path to 20s and explicitly monitors the Arduino loop
//     task. The loop task is fed at the start of every loop() pass. The sensor
//     task remains monitored by the existing sensor-task feed.
//   - Does not access private lwIP TCP-PCB internals; socket-level diagnostics
//     are the stable boundary available to this Arduino build.
//
// v3.9.80 CONNECTED-SESSION RX RECOVERY + HARDER WIFI TEARDOWN
//   - Adds active recovery for an established Blynk session whose RX side has
//     stopped progressing while WiFi remains associated. RX silence remains
//     diagnostic at 10s/20s/25s; at 30s a Blynk-only transport reset is made.
//     WiFi is deliberately untouched by this RX recovery path.
//   - Adds a 60s recovery holdoff to prevent repeated Blynk-only churn.
//   - Fixes WiFi recovery teardown for Core 3.x: plain WiFi.disconnect() could
//     leave the STA driver in status=6 / CONNECTING. The intentional teardown
//     now powers WiFi off while retaining credentials; wifiService() waits a
//     bounded settle interval, then re-enables STA before WiFi.begin().
//   - Existing sensor, GPS, INAV, Blynk, watchdog and diagnostic logic is kept.
//
// v3.9.98 SIMPLE STOCK-BLYNK RECOVERY
//   - Simplifies the stock-session supervisor to four explicit states: IDLE,
//     CONNECTING, ONLINE and RECOVERY. Only blynkStockEnterRecovery() owns
//     Blynk.disconnect(), stock-client stop and Blynk.config().
//   - Removes the unreliable blynkLastRxMillis stale-session watchdog from
//     stock recovery. Field logs showed rxAge increasing during an otherwise
//     functioning stock session, so it is retained as diagnostics only.
//   - A slow virtualWrite() only latches recovery after write() returns; the
//     outer supervisor then performs exactly one clean Blynk-only reset.
//   - Adds a 2s RECOVERY watchdog that can return the supervisor to IDLE if an
//     unexpected library interaction leaves the state there. WiFi is never
//     reset by this Blynk-only path.
//   - A normal Blynk disconnect and a TX stall converge on the same recovery
//     path, then a fresh stock Blynk.run() is attempted after a 150ms settle.

// v3.9.97 STOCK TX RECOVERY OWNERSHIP FIX
//   - Field log confirmed the stock WiFi/Blynk session can reach a ~10.01s
//     NetworkClient.write() stall, after which two immediate "Disconnected"
//     messages appeared but the supervisor never produced its recovery log.
//   - Root cause in the sketch: blynkTxService() called Blynk.disconnect()
//     inside the TX-stall handler, while the outer stock supervisor also
//     called blynkStockEnterRecovery(), which called Blynk.disconnect() again.
//     Recovery now has ONE owner: blynkStockEnterRecovery(). The TX service
//     only latches blynkStockTxRecoveryPending after write() returns.
//   - Added explicit RECOVERY ENTER/READY logging around the single disconnect
//     and config reset so the next field run proves whether recovery actually
//     executes after the 10s stall. WiFi is never touched by this path.
//
// v3.9.116 STOCK BLYNK HEARTBEAT RECOVERY — EXACT 0.6.1 LIBRARY FIX
//   - Keeps stock BlynkSimpleEsp32/WiFiClient transport unchanged.
//   - Uses the exact supplied Blynk 0.6.1 behavior: heartbeat timeout is
//     BLYNK_HEARTBEAT + 3*BLYNK_TIMEOUT_MS, and inbound RESPONSE/PING traffic
//     updates the library's private lastActivityIn.
//   - Does NOT add an application-level PING. Blynk.run() remains the sole
//     protocol heartbeat owner.
//   - Removes the observed-RESPONSE-age metric as an ACTIVE TX gate. The
//     passive MSG_PEEK observation can miss protocol traffic that Blynk.run()
//     has already consumed, so it is diagnostic only and must not suppress
//     valid telemetry on a healthy session.
//   - Adds the missing stock retry-failure accounting. After three failed
//     disconnected Blynk.run() attempts, the sketch performs ONE clean
//     Blynk.disconnect()+Blynk.config() refresh, without touching WiFi.
//     This resets the exact v0.6.1 protocol state machine without creating a
//     second connection owner.
//   - The clean refresh is rate-limited to 30s and is followed by the normal
//     stock Blynk.run() retry cadence.
//   - WiFi recovery remains completely separate from Blynk recovery. A Blynk
//     heartbeat/session loss does not call WiFi.disconnect(), WiFi.begin(),
//     or WiFi.reconnect(). WiFi is only recovered when the WiFi state machine
//     itself reports an actual association failure.
//   - Automatic WiFi driver reconnect is used only after two consecutive
//     20s association failures; the custom WiFi state machine remains the
//     normal owner once connectivity is restored.
//
// Firmware version - printed on boot and sent to Blynk for tracking
// v3.9.112 STOCK BLYNK STALE-RESPONSE TX GUARD
//   - Starts from the v3.9.111 final forensic experiment after confirming that
//     the deterministic V61 payload can run for a long time with 2-12ms writes,
//     while the failure pattern is a loss of newly observed Blynk RESPONSE
//     traffic followed by the stock heartbeat timeout.
//   - Adds a per-session observed-RESPONSE freshness guard for application TX.
//     When no new five-byte RESPONSE header has been observed for 45s, queued
//     application virtualWrite traffic is quarantined. Blynk.run() continues
//     normally, so stock heartbeat/handshake traffic is never intentionally
//     suppressed by this guard.
//   - At 30s, 45s and 60s of observed-response silence, the guard emits one
//     explicit diagnostic. The 60s point is the BLYNK_HEARTBEAT boundary and
//     is intentionally observational only; the stock library remains the sole
//     owner of heartbeat timeout/session closure.
//   - A new session seeds the freshness clock from the session-start handshake
//     observation, avoiding a false immediate TX quarantine.
//   - The existing 1500ms TX circuit breaker remains as a second line of
//     defence, but the new guard is designed to prevent entering the known
//     ~10s NetworkClient.write() stall once the server-response path has
//     already gone stale.
//   - No custom/raw Blynk transport, application PING, WiFi teardown, or
//     sketch-initiated Blynk disconnect is added. Stock BlynkSimpleEsp32 remains
//     the transport and Blynk.run() remains the session/heartbeat owner.
//
// Firmware version - printed on boot and sent to Blynk for tracking
// v3.9.145 NORMAL PUBLISHING + NON-DESTRUCTIVE TX BACKOFF
//   - Restores normal ESP32 802.11b/g/n operation after the legacy-b/g A/B test.
//   - A slow but completed Blynk write pauses application telemetry for 8s;
//     it no longer tears down a session that still reports connected.
//   - A failed independent TCP probe also holds telemetry without resetting the
//     live Blynk session; Blynk.run() remains the sole connected-session owner.
//   - Replaces the V61-only forensic timer with coherent V61/V62, V4, smoothed
//     dashboard/diagnostic round-robin publishers, and the map publisher.
// v3.9.146 NON-BLOCKING CROSS-CORE SENSOR SNAPSHOT
//   - Core-1 Blynk publishers never wait for the Core-0 sensor mutex.
//   - Each publisher attempts an immediate snapshot copy; if Core 0 is committing
//     data at that instant, Core 1 reuses its last complete cached snapshot.
//   - Adds runtime task/core reporting so the deployed build proves its affinity.
// v3.9.149 SOFT STA RECOVERY AFTER BOUNDED BLYNK WRITE FAILURE
//   - v3.9.148 proved the write bound: the old 10012ms NetworkClient stall
//     returned in 756ms. Its Blynk escalation then used the generic hard
//     WiFi.disconnect(true,false) path, which powered the radio off and led
//     to status=6/reason=2 and repeated CONNECT_TIMEOUT power cycles.
//   - Blynk TCP-stack escalation now performs WiFi.disconnect(false,false):
//     the STA driver/radio and credentials remain loaded while the association,
//     DHCP/netif and all TCP sockets are rebuilt after a 2s settle interval.
//   - While that targeted recovery is active, later 20s association timeouts
//     also use the soft path instead of repeatedly powering the radio off.
//     The flag clears only after a new Blynk login succeeds.
//
// v3.9.150 DEDICATED CORE OWNERSHIP + ONE-WAY SENSOR SNAPSHOT
//   - Requires Arduino loop and Arduino events on Core 0, alongside the ESP-IDF
//     WiFi and lwIP tasks that are already pinned there by Arduino-ESP32 3.3.12.
//   - Moves sensorTask to Core 1. Sensor driver initialization (ADC, I2C,
//     GPS UART, PPS ISR, I2S, OLED and CO PWM) also executes from Core 1.
//   - Publishes a complete SensorData value through a one-slot overwrite queue.
//     Blynk only peeks that immutable snapshot; it never waits on the sensor
//     mutex and never invokes a sensor driver.
//
// v3.9.151 STRICT CORE-OWNERSHIP REFACTOR
//   - Core 1 is now the sole runtime owner of SensorData (`sd`). Core 0 never
//     writes WiFi flags or engineering text into that structure; connection
//     state is overlaid only on Core 0 after a queued sensor snapshot is read.
//   - Adds a second one-slot Core1->Core0 queue for sensor health/performance
//     diagnostics. RTC checkpoints, Blynk diagnostic V-pins and connection
//     health logs consume that snapshot instead of reading sensor globals.
//   - Adds a zero-wait Core1->Core0 sensor-log queue. Sensor code formats its
//     own messages on Core 1, while only Core 0 touches the physical Serial
//     object. A full queue drops a log record; it can never delay acquisition
//     or make the WiFi/Blynk loop wait on Arduino Serial's internal lock.
//   - The only cross-core data paths are bounded FreeRTOS queue copies, atomic
//     task-health markers and the deliberate reset/watchdog mechanisms.
//
// v3.9.148 BOUNDED STOCK-PROTOCOL WRITE + TCP-STACK ESCALATION
//   - Keeps the stock Blynk 0.6.1 protocol and WiFiClient connect/read path,
//     but overrides NetworkClient::write() locally. Arduino-ESP32 3.3.12 uses
//     100 x 100ms retry slots irrespective of setTimeout(), which exactly
//     explains the measured 10012ms publisher stall. The firmware now gives
//     one complete atomic Blynk frame a hard 750ms write budget.
//   - Corrects WiFiClient::setTimeout() units for Arduino-ESP32 3.3.12:
//     milliseconds, not seconds. BLYNK_TIMEOUT_MS is passed directly.
//   - Restores Nagle/default batching; forcing TCP_NODELAY added avoidable
//     small-packet pressure and did not prevent the field failure.
//   - After a previously-online session cannot reconnect for 20s and has
//     already completed one clean Blynk refresh, performs one intentional,
//     non-blocking WiFi interface rebuild. This clears the poisoned ESP32
//     lwIP/netif state while retaining credentials and without rebooting.
//   - A WiFi re-association now explicitly puts BlynkProtocol back into
//     CONNECTING with Blynk.connect(0); config() alone cannot do that after
//     Blynk.disconnect() in legacy Blynk 0.6.1.
//
// v3.9.147 FAST SINGLE-SOCKET BLYNK RECOVERY
//   - Removes the separate five-second TCP reachability probe from connected-
//     session admission; it tested a different socket and added server churn.
//   - Retries a lost local-LAN session after 1s, with bounded login backoff and
//     much shorter clean-refresh holdoffs.
//   - Reduces post-connect telemetry holdoff to 2s and slow-write backoff to 3s.
//   - Retains the 3s WiFiClient timeout: the field log proved a 1.952s write
//     can still complete, so a 1s timeout would create a new disconnect.
// v3.9.214 EVENT-DRIVEN TCP QUIET RECOVERY
//   - Root cause in the v3.9.213 trace: every failure was an exact application
//     checkpoint RESPONSE loss. While that checkpoint was pending, the Blynk
//     library correctly suppressed its ordinary protocol PING, but the bulk
//     HTTP isolation gate watched only that suppressed protocol PING. HTTP
//     therefore kept sending/retrying on the same degraded ESP32 WiFi/lwIP
//     path (diagnostic `isolations=0`) until both sockets failed.
//   - Treat either exact proof (application checkpoint or protocol PING) as a
//     health threat after 3 seconds. Close/pause HTTP immediately and allow
//     only the already-buffered Blynk/TCP retransmission to use the path.
//   - Extend the application/generic sequence-hole deadline to 70 seconds.
//     Earlier packet captures measured successful ESP32 TCP retransmission at
//     47-58 seconds; the old 25-second deadline destroyed recoverable sockets.
//   - An open HTTP circuit is no longer proof by itself. A circuit lasts 90
//     seconds and the trace showed it was still causing MCU resets with only
//     one recovered failure 49-63 seconds old. Full shared-path recovery now
//     requires a fresh unresolved streak of at least two HTTP failures.
//   - V4 remains 1 second, V61/V62 remain an immutable timestamp pair, normal
//     802.11b/g/n remains enabled, and Core 1 remains free of network calls.
#define FIRMWARE_VERSION "3.9.214"

// v3.9.198 EARLY SESSION-PATH RECOVERY + FIRST-DISCONNECT OWNERSHIP
//   - The v3.9.197 trace lost RESPONSE 691 while WiFi, heap and both cores were
//     still healthy. The exact application checkpoint then remained pending for
//     110 seconds before the AP finally disassociated the otherwise idle STA.
//   - Restore the evidence-based 25-second deadline for an exact application
//     checkpoint only. Recovery rotates the Blynk socket; it does not reset the
//     radio or MCU. The conservative 150-second generic deadline is retained.
//   - Arduino-ESP32 3.3.12 always performs one automatic retry on the first
//     lifetime disconnect even when auto-reconnect is disabled. Do not issue a
//     competing esp_wifi_connect() for that first event; manual retries begin
//     only after a subsequent disconnect event from the failed core retry.

// v3.9.197 STRING QUEUE TERMINATION SAFETY
//   - Replace three accidental multi-character '\\0' constants with the NUL
//     byte '\0'. The old value could leave string virtual-pin queue entries
//     unterminated, permitting stale bytes or an out-of-bounds read in the
//     ESP32-only string telemetry path.

// v3.9.196 SINGLE-OWNER WIFI RECOVERY
//   - The v3.9.195 field trace is a real STA/AP loss, not a Java/Blynk stall:
//     reason 4 (inactivity), 2 (authentication expiry), 16 (group-key update
//     timeout) and 203 (association failure) occur while IP is 0.0.0.0.
//   - Restore ESP-IDF's six-second STA beacon-inactivity threshold. The former
//     20-second override prolonged a dead association and its one-way TCP
//     stream before the driver emitted STA_DISCONNECTED.
//   - Defer each esp_wifi_connect() until 500ms after its disconnect event and
//     issue at most one attempt per event. This removes calls made while the
//     driver still reports "sta is connecting".
//   - If bounded fast reauthentication fails, explicitly stop/reset the STA,
//     wait for driver settle, and then perform one unconstrained WiFi.begin().
//     Never call WiFi.begin() or WiFi.reconnect() over an active attempt.
//   - Do not pin recovery to a cached BSSID/channel after an AP/security loss;
//     a full scan tolerates AP channel changes, roaming and key rotation.

// v3.9.195 SERVER-STALL CONTAINMENT + PRESERVE THE RECOVERABLE TCP STREAM
//   - The v3.9.194 field trace disproves the remaining 25-second socket rule.
//     After the established stream stopped receiving, fresh TCP/login attempts
//     also failed for 117 seconds while WiFi stayed associated at -52..-56 dBm
//     with zero disconnect events, EAGAINs, write timeouts or socket errors.
//     A later socket authenticated without WiFi recovery. This is endpoint/
//     server unavailability, not evidence that the ESP32 stream alone is bad.
//   - Application publishing still stops at the existing 6/10-second inbound
//     brake, bounding queued work immediately. Keep the authenticated socket
//     for 150 seconds before a socket-only rotation when WiFi and TCP still
//     claim connected. This covers the observed 117-second whole-endpoint pause
//     and remains below BlynkProtocol's independent 189-second hard deadline.
//   - Application checkpoints, protocol PINGs and the generic inbound fallback
//     use the same server-stall grace. A real TCP close or WiFi event continues
//     through the immediate existing recovery path; no fault is ignored.
//   - The paired blynk.service update removes the Pi's low MemoryHigh reclaim
//     cliff and uses Java 21 generational ZGC so prevention occurs at the server
//     rather than making the ESP32 churn sockets during a stop-the-world pause.

// v3.9.194 ATOMIC CANARY PROOF + RETRANSMISSION-AWARE RECOVERY
//   - The v3.9.193 field trace keeps WiFi at -54..-56 dBm with zero disconnect
//     events, zero EAGAIN/write timeouts and fast heartbeat-only responses. The
//     repeated failure is instead created after a V62 frame and a separate
//     five-byte partial-batch PING: healthy examples already take 4.7-6.8s,
//     then the 12s application deadline closes two otherwise viable sockets.
//   - A recovery canary now forces its PING into the SAME atomic write as its
//     HARDWARE frame. It cannot be retained by Nagle behind the unacknowledged
//     frame, and one retransmission repairs both the data and its proof.
//   - Disable the two-second standalone partial-batch PING. Normal operation
//     retains four-frame atomic checkpoints, while the existing independent
//     ten-second protocol PING proves a quiet tail without another tiny write.
//   - An application-only checkpoint delay now gets the measured 65-second TCP
//     retransmission window. A protocol-owned PING still has a separate 25s
//     deadline. This stops application/server backpressure from being mislabeled
//     as immediate transport death while preserving bounded socket recovery.
//   - Changes remain behind ESP32-only opt-in macros; an ESP8266 using the same
//     Blynk library follows the original code path unchanged.

// v3.9.193 LINK-LOSS PREVENTION + NON-DESTRUCTIVE STA RECOVERY
//   - The v3.9.192 field trace starts with an accepted-by-lwIP Blynk stream
//     becoming one-way as RSSI falls to -69..-71 dBm, then later shows a real
//     STA association failure. This is a common lower-link degradation, not a
//     malformed V-pin or Blynk quota event.
//   - In marginal RF, retain only V4 and the authoritative V61/V62 pair and
//     pace application frames at 1 Hz. Protocol PING and Blynk.run() remain
//     unrestricted. Hysteresis prevents mode chatter.
//   - Extend the ESP-IDF station beacon-inactivity window from its 6-second
//     default to 20 seconds, allowing TCP retransmission to ride through short
//     beacon/interference fades instead of destroying the association.
//   - Preserve the last good AP BSSID/channel and use it for the first three
//     reassociation attempts, avoiding a full-channel scan on the known LAN.
//   - Retry every failed STA_DISCONNECTED association event non-destructively.
//     The old path ignored follow-up events, then powered the radio off after
//     one failed attempt and repeated 20-second disconnect/begin cycles.
//   - Preserve the outage's first disconnect reason; failed association reason
//     4 can no longer overwrite the trigger that began the outage.
//   - A hard driver rebuild remains as a last resort after six failed attempts;
//     Blynk faults alone still never reset WiFi or the MCU.

// v3.9.192 APPLICATION-CANARY RECOVERY + FLAP-STATE FIX
//   - The v3.9.191 field trace proves replacement sockets can pass 30 seconds
//     of heartbeat-only traffic and then become one-way as soon as application
//     publishing resumes. Clearing the recovery generation at heartbeat proof
//     reset every such failure to socket trial 1/3, so cooldown was unreachable
//     and an impaired server/path could flap online/offline indefinitely.
//   - Heartbeat proof now transitions to an application canary phase instead
//     of full publishing. Eight application frames are admitted one at a time;
//     each must receive its exact partial-batch PING checkpoint before the next
//     frame. This covers two former four-frame batches without a burst and
//     identifies the last virtual pin in the serial log if a failure repeats.
//   - Only after all eight checkpoints pass are normal publishers released and
//     the recovery generation cleared. Canary/heartbeat failures retain and
//     advance the socket-trial counter, making the existing three-trial server
//     cooldown reachable. Brief logins no longer erase outage backoff state.

// v3.9.191 CAUSAL STREAM PROBES + PARTIAL-BATCH CHECKPOINT
//   - Replaces the scripted recovery-only simulation with a 20-minute model
//     in which Blynk frames consume TCP sequence space, correlated loss drives
//     retransmission, and the server cannot answer a PING behind a missing
//     ordered byte. This reproduces the supplied per-connection failure while
//     a new connection remains viable.
//   - The v3.9.190 trace stopped at application batch progress 3/4. With no
//     checkpoint outstanding, only the conservative 25-second generic inbound
//     deadline could identify the dead stream. Flush an incomplete batch with
//     an exact PING after two idle seconds and track its RESPONSE explicitly.
//   - Track protocol-owned heartbeat PING IDs too. An exact application or
//     protocol probe unanswered for 12 seconds rotates only the Blynk socket;
//     the 25-second generic fallback remains. No periodic reconnect, WiFi
//     teardown, or communication-fault MCU restart is introduced.

// v3.9.190 NO COMMUNICATION-FAULT MCU RESTARTS
//   - The v3.9.189 field run proved that the three-socket sequence-hole limit
//     can be exhausted during a multi-minute server outage while WiFi remains
//     associated and every earlier application checkpoint is verified. A full
//     ESP32 restart cannot repair that remote failure domain. After three bad
//     replacement sessions, reset only BlynkProtocol, hold for 30 seconds, and
//     continue heartbeat-only verification without rebooting the sensor.
//   - Reason 2 is ESP-IDF WIFI_REASON_AUTH_EXPIRE. On an unexpected disconnect,
//     issue one owner-managed esp_wifi_connect() reauthentication attempt before
//     tearing down the STA. This follows Espressif's reconnect path and avoids
//     destroying otherwise recoverable driver/AP authentication state.
//   - Five failed association attempts now rebuild only the WiFi radio/driver
//     and restart the bounded association state machine. Blynk, WiFi and login-
//     only failure paths never call esp_restart(). Core-hard-stall, sensor-
//     recovery and explicit user reset protections remain unchanged.

// v3.9.189 BATCHED END-TO-END ACK + SERVER-PRESSURE PREVENTION
//   - v3.9.188 successfully avoided the exact 128-byte trigger 276 times and
//     extended the healthy session, but a later 133-byte V62 still preceded a
//     25-second one-way interval. The following replacement login attempts
//     also timed out for about 44 seconds while WiFi/RSSI remained unchanged,
//     identifying temporary peer/event-loop unavailability rather than an
//     ESP32 association failure or another fixed-length-only defect.
//   - Official legacy-server source shows every PING is decoded as a separate
//     message and immediately writeAndFlush()es a RESPONSE on the same Netty
//     event loop that synchronously processes HARDWARE messages. v3.9.188 sent
//     794 hardware frames plus 1,070 ACK PINGs in the supplied session.
//   - Keep ordered end-to-end verification but checkpoint one bounded batch of
//     four application frames. A matching PING RESPONSE proves all four TCP-
//     ordered frames, limits unverified exposure to four, and cuts protocol
//     message/response allocation substantially. Any bare or combined write
//     that would equal 128 bytes is still promoted/padded with valid PINGs.

// v3.9.188 EXACT 128-BYTE ATOMIC-WRITE PREVENTION
//   - The v3.9.187 field trace identifies a deterministic byte boundary, not
//     a timing or reconnect failure. Healthy V61 values of length 111 produce
//     a 127-byte atomic HARDWARE+PING write. The first length-112 V61 produces
//     exactly 128 bytes and is the first unanswered write. Every replacement
//     session then reproduces the failure on its first length-112 V61 while
//     smaller V4+PING and heartbeat-only traffic remains bidirectional.
//   - Preserve the end-to-end ACK barrier, but when its complete atomic write
//     would be exactly 128 bytes append a second, valid Blynk PING and wait for
//     that final PING's RESPONSE. The resulting 133-byte write avoids the
//     field-proven boundary without malformed padding, payload truncation, or
//     a second TCP owner. This opt-in path remains ESP32-only.
//   - Report how often the 128-byte boundary was avoided so the next field log
//     can directly confirm that the prevention path was exercised.

// v3.9.187 END-TO-END APPLICATION ACK BARRIER
//   - The v3.9.186 field trace isolates the initiating condition below the
//     reconnect state machine: replacement sockets exchange login/info and
//     repeated PING/RESPONSE frames while application TX is prohibited, then
//     become one-way only after virtualWrite traffic is released. WiFi remains
//     associated, RSSI/heap are stable, and lwIP accepts every local write.
//   - Opt into a BlynkProtocol transport barrier for this ESP32 only. Every
//     BLYNK_CMD_HARDWARE frame is followed by a unique PING header in the SAME
//     atomic transport write. TCP ordering means the matching RESPONSE proves
//     that all bytes of the preceding virtualWrite reached and were parsed by
//     the server. No later application frame is admitted until that proof.
//   - This bounds exposure to one unverified application frame and immediately
//     stops adding bytes behind a missing TCP segment. Protocol processing and
//     recovery remain live. ESP8266 sketches do not define the opt-in macro and
//     therefore compile the unchanged Blynk 0.6.1 path.

// v3.9.186 HEARTBEAT-ONLY REPLACEMENT PROBATION + SOCKET-ONLY ESCALATION
//   - The overnight v3.9.185 trace exposed that the replacement-session trial
//     was only a label: the ordinary two-second reconnect quarantine expired
//     while the 30-second verification flag was still set, allowing a queued
//     application burst onto the unverified TCP stream. A recovery trial now
//     suppresses both timer producers and the TX service. Blynk.run() remains
//     active, so only login/info and protocol-owned PING/RESPONSE traffic test
//     the replacement path.
//   - A Blynk-only sequence hole no longer stops/deinitializes WiFi. The field
//     trace showed that RADIO_OFF converted a working association into repeated
//     association failures and ultimately forced an MCU reset; ESP-IDF also
//     defines esp_wifi_stop() as freeing the STA control block. Recovery now
//     rotates only the owner-managed Blynk socket, up to three independently
//     authenticated heartbeat-only trials.
//   - TCP/login failures do not prove a poisoned local stack and therefore use
//     the existing bounded retry/holdoff without touching WiFi or resetting the
//     MCU. Only three authenticated trials reproducing the precise one-way
//     predicate use the proven process-wide reset fallback.

// v3.9.185 TRUE INBOUND CLOCK + LOWER EXPOSURE + FAST SOCKET RECOVERY
//   - The v3.9.184 field log proves the 6..10s pre-PING quiet window works, but
//     also proves that quiet time cannot repair an already missing TCP byte.
//     The last healthy RESPONSE was followed by locally accepted writes and six
//     unanswered PINGs; WiFi stayed associated and a replacement Blynk socket
//     authenticated in about one second without a WiFi or MCU reset.
//   - Rotate only the poisoned Blynk socket after 25s of real protocol-inbound
//     silence. The largest healthy gap in the supplied run was 20.042s, leaving
//     a bounded margin while cutting this measured outage from ~66s to ~26s.
//     A replacement-session failure still follows the existing driver-then-MCU
//     escalation ladder; generic Blynk failures still cannot reset WiFi.
//   - Reduce independent application transmissions from the v3.9.180 nominal
//     1.7/s to about 0.88/s: V61/V62 and dashboard cadence 4s, diagnostic/V4
//     cadence 15s, with 500ms pacing. The 6s send window now has substantial
//     service headroom instead of arrival rate almost exactly matching capacity.
//   - Replace the legacy session-age "RX proxy" with BlynkProtocol's exported
//     lastInboundActivity(). The old diagnostic climbed for the entire session
//     even while fresh RESPONSE headers arrived, obscuring the true boundary.

// v3.9.184 REAL PRE-PING QUIET WINDOW + STAGED PATH RECOVERY
//   - The v3.9.183 log shows V61, V62 and V29 transmitted immediately before
//     the first unanswered 10-second protocol PING. The claimed post-PING quiet
//     state was never armed because its legacy heartbeat helper is not called;
//     the patched BlynkProtocol owns PING generation.
//   - Stop application writes when protocol inbound age reaches 6 seconds. This
//     reserves roughly four application-silent seconds before each normal 10s
//     PING while Blynk.run(), PING and TCP retransmission continue unchanged.
//     The existing 10s stale log remains fault telemetry, not the first brake.
//   - Do not power-cycle WiFi at the first 65s sequence-hole decision. Rotate
//     only the Blynk socket first. Escalate to one full driver rebuild only if
//     that replacement path records three real TCP connect failures or itself
//     reaches the 65s one-way predicate. Reset the MCU only after the rebuilt
//     path also fails. This avoids the observed reason=4 association-expiry
//     delay when a narrower socket recovery is sufficient.

// v3.9.183 FIX CROSS-CORE SUPERVISOR RACE + MUTUAL VIABILITY HANDSHAKE
//   - The v3.9.182 log proves its software resets were false positives:
//     CORE0 HARD STALL reported age=4294967ms at only 10.6s uptime. The Core-1
//     monitor sampled micros() before the Core-0 progress timestamp; a progress
//     update between those reads made unsigned subtraction wrap near 2^32 us.
//   - Read the shared progress timestamp first and micros() second. A concurrent
//     Core-0 update can now only make the computed age slightly conservative;
//     it can never place the timestamp in the monitor's future.
//   - Add bidirectional viability. Core 0 requests a cooperative Core-1 sensor
//     bus/ENS/BMI recovery after 8s without heartbeat and requires an ack within
//     8s. Only a missing ack escalates to full reset. Core 1 continues to use a
//     full-chip reset for a real 30s communications stall because Arduino,
//     WiFi and lwIP cannot be safely restarted as an isolated CPU core.

// v3.9.182 INDEPENDENT HARD-STALL RECOVERY + SINGLE BLYNK REFRESH
//   - The Core-1 communications monitor previously printed every second during
//     a Core-0 stall. If Core 0 held the UART or driver lock, that print could
//     also block the monitor, leaving no independent recovery owner.
//   - The monitor is now lock-free: it reads scalar progress state, records the
//     failing stage in RTC memory, and restarts after 30 seconds without using
//     Serial, WiFi, Blynk, a mutex or a queue. The 20-second task WDT remains
//     primary; this is an independent fallback for the observed silent hang.
//   - Perform at most one Blynk disconnect/config refresh per outage. After
//     three more bounded failures, preserve protocol state and use a 30-second
//     quiet holdoff instead of repeatedly restarting Blynk.

// v3.9.181 REBUILD A FALSE-CONNECTED WIFI DRIVER BEFORE MCU RESET
//   - The v3.9.180 field run proves traffic reduction alone is not sufficient:
//     the first session became one-way with strong RSSI, no WiFi disconnect
//     event and stable heap, then six of seven fresh TCP connects failed while
//     WiFi.status() continued to report WL_CONNECTED.
//   - A confirmed 65-second sequence hole now rebuilds the owning WiFi driver,
//     netif and lwIP attachment once, instead of rotating Blynk sockets on the
//     same demonstrably poisoned path. Arduino WiFi.mode(WIFI_OFF) performs the
//     full driver deinit; the existing two-second settle then recreates STA.
//   - Preserve the recovery-verification trial across this intentional WiFi
//     reassociation. Clear it only after 30 seconds with fresh Blynk inbound
//     traffic. If the rebuilt path reaches the same precise failure predicate,
//     retain the proven MCU-reset fallback rather than loop driver rebuilds.

// v3.9.180 REDUCE TCP/802.11 TRANSMISSION PRESSURE (PREVENTION)
//   - The v3.9.179 field run remained computationally healthy: no WiFi event,
//     RSSI -54..-55dBm, stable heap, no connected-session Blynk.run stall, and
//     application writes <=25ms. The stream nevertheless developed another
//     ESP32->server TCP sequence hole after about 22 minutes, and its replacement
//     stream repeated the failure. Recovery works, but is only the safety net.
//   - Reduce the controllable exposure: independent transmissions. The previous
//     schedule offered about 3.7 application commands/s. This profile offers
//     about 1.7/s (54% fewer): a coherent V61/V62 pair every 2s, one dashboard
//     pin every 2s, one diagnostic pin every 10s, and V4 every 10s.
//   - Increase transport pacing from 200ms to 350ms, reducing the application
//     scheduler ceiling from 5/s to 2.86/s. All virtual pins and complete
//     coherent records are preserved; only their publication cadence changes.

// v3.9.179 VERIFY TCP-SEQUENCE-HOLE RECOVERY
//   - Field evidence from v3.9.178 shows that closing the poisoned Blynk socket
//     can produce a replacement TCP session which accepts HW_LOGIN but remains
//     one-way: the device prints Ready and local writes, while the server/device
//     dashboard remains offline and no post-login protocol response arrives.
//   - Keep the first 65-second recovery narrowly scoped to the Blynk socket.
//     Mark that replacement session as a recovery trial and require 30 seconds
//     of operation with fresh inbound protocol traffic before declaring it good.
//   - If the trial reaches the same 65-second inbound-silence boundary, restart
//     the MCU. Earlier field tests established that this is the reset domain
//     which clears the process-wide lwIP/TCP state; repeating login attempts on
//     the same poisoned state only prolongs the outage.

// v3.9.178 RECOVER THE ACTUAL ON-WIRE TCP SEQUENCE HOLE
//   - Server-side packet capture proves the missing Blynk RESPONSE is a
//     consequence, not the initiating fault.  The server ACK remains fixed at
//     the first missing ESP32 TCP byte while later ESP32 segments arrive out of
//     order.  TCP ordering therefore prevents the server from parsing the
//     later Blynk PING, so no PONG can be generated until the hole is repaired.
//   - The previous recovery test required awaitingPeerConfirmation(), which is
//     armed only after local send-buffer backpressure/EAGAIN.  In the captured
//     failure every write was accepted by lwIP (pending=0, EAGAIN=0), making
//     that test permanently false even while the peer ACK was stuck.
//   - Detect the wire-visible condition instead: this session was authenticated
//     and received post-login traffic, WiFi has had no disconnect event, the
//     Blynk TCP socket still claims connected, but BlynkProtocol has received
//     no response for 65 seconds.  Sixty-five seconds is beyond the longest
//     observed natural retransmission repair (58 seconds).  Close through the
//     socket owner and explicitly restore BlynkProtocol to CONNECTING, thereby
//     discarding only the poisoned TCP sequence space.  WiFi is not torn down.
//   - Keep the 15-second application-TX brake and 10-second protocol PINGs:
//     they first give normal TCP repair a quiet interval and provide the exact
//     end-to-end evidence used by this supervisor.

// v3.9.177 REMOVE CONFLICTING 30-SECOND KERNEL TCP WATCHDOG
//   - The larger v3.9.176 report proves the socket itself becomes disconnected
//     around 30 seconds after inbound traffic stops (tcpBeforeRun=0), despite
//     the Blynk protocol deadline being 189 seconds and WiFi having zero
//     disconnect events. This matches the socket policy configured here:
//     SO_KEEPALIVE with 15s idle + three 5s probes closes at about 30 seconds.
//   - That kernel watchdog silently overrode the evidence-based retransmission
//     window and killed the same stream the 15s application-TX brake was trying
//     to preserve. Disable SO_KEEPALIVE on the Blynk socket. BlynkProtocol is
//     now the single liveness owner: 10s PING cadence, 189s failure deadline.
//   - This does not disable TCP retransmission or TCP ACK processing. It only
//     removes the redundant keepalive close timer. WiFi and ESP8266 behavior
//     are unchanged; the macro-scoped BlynkProtocol defaults remain stock for
//     sketches that do not opt into the separated cadence/deadline.

// v3.9.176 PRESERVE TCP RETRANSMISSION + STALE-RX TRAFFIC BRAKE
//   - The packet capture already proved that a missing ESP32->server TCP segment
//     can take 47-58 seconds to arrive by normal exponential retransmission.
//     v3.9.170 regressed the protocol deadline to 19 seconds merely to copy the
//     ESP8266 defaults, guaranteeing that Blynk destroyed a recoverable ESP32
//     stream before TCP could repair it. Restore the evidence-based 180-second
//     heartbeat deadline (189 seconds including the protocol timeout margin).
//   - Keep one BlynkProtocol heartbeat owner, but separate its probe cadence
//     from its failure deadline: PING every 10 seconds and retry every 10
//     seconds. A returned RESPONSE proves the ordered TCP stream recovered.
//   - Once internal Blynk inbound activity is stale for 15 seconds, pause only
//     application virtualWrite traffic. Blynk.run/PING continue, so the TCP
//     retransmission queue can drain instead of receiving several new frames
//     per second. Publishing resumes automatically on fresh inbound traffic.
//   - This prevents premature reconnect churn seen in v3.9.175, which reduced
//     free heap from ~139KB to 128KB, 111KB and 84KB as closed TCP sessions
//     remained in retransmit/FIN/TIME_WAIT state. No periodic healthy pause is
//     introduced, and ESP8266 behavior remains unchanged.

// v3.9.175 SOCKET-OWNER-SAFE RECOVERY + PROVEN LOGIN-ONLY FAIL-SAFE
//   - The v3.9.174 field log proves that WiFi remains associated (zero WiFi
//     disconnect events, good RSSI) while Blynk repeatedly accepts HW_LOGIN and
//     then returns no post-login protocol traffic until the 19s heartbeat
//     deadline. Local "<[...]" lines prove only that bytes reached the local
//     socket API; they do not prove that the server received them.
//   - Removes the v3.9.172 direct tcp_abort() walk of lwIP PCB lists. The Blynk
//     connection is a BSD socket/NetworkClient connection, so its netconn and
//     PCB must be released by its owning socket via Blynk.disconnect()/stop().
//     Aborting the raw PCB behind that owner can leave socket bookkeeping stale
//     and make subsequent Blynk-only reconnects less capable than esp_restart().
//   - Records whether each authenticated session receives any NEW inbound
//     protocol activity after login. Three login-only heartbeat deaths,
//     while WiFi stays WL_CONNECTED with the same disconnect-event sequence,
//     trigger the already proven full software reset. A single transient cannot
//     reset the device, and a healthy post-login response clears the streak.

// v3.9.174 SINGLE WIFI OWNER + TCP BATCHING RESTORATION
//   - The v3.9.173 field log contains seven ESP-IDF reason=36
//     (WIFI_REASON_STA_LEAVING) events.  This is a local STA leave/reset, not
//     poor RSSI or an AP-side rejection.  The sketch had accidentally restored
//     WiFi.setAutoReconnect(true) while wifiService() still owned reconnects.
//     Return to one owner: the Arduino WiFi auto-reconnect path stays disabled
//     and wifiService() alone handles real STA_DISCONNECTED events.
//   - A failed Blynk/TCP reconnect no longer powers off or reassociates WiFi.
//     Blynk recovery is restricted to the Blynk socket/protocol. WiFi teardown
//     is reserved for an
//     actual WiFi event/state failure.
//   - Restore Nagle batching on the Blynk socket.  TCP_NODELAY caused every
//     small virtualWrite to become a separate TCP/802.11 transmission and did
//     not prevent the missing-ACK failure.  Keepalive remains enabled for
//     bounded dead-peer detection.  These changes are local to this ESP32 INO;
//     the shared Blynk library and ESP8266 path are unchanged.

// v3.9.173 FIX FALSE 90-SECOND DATA-PATH RECOVERY
//   - v3.9.172 field evidence showed session #2 receiving fresh Blynk RESPONSE
//     frames every ~10 seconds (msgIds 19..25), yet the transport remained
//     awaitingPeer=1 and the sketch deliberately destroyed that healthy
//     session at 90 seconds.
//   - Cause: patched BlynkProtocol sends its own ordered PING immediately after
//     deferred bytes drain, but the sketch-side explicit heartbeat function is
//     disabled and therefore never arms peerProbeMsgId_. confirmPeerProgress()
//     consequently rejected every valid RESPONSE.
//   - Once deferredCount_ is zero, accept the first fresh RESPONSE when no
//     explicit probe ID is armed. TCP ordering makes this safe: the protocol's
//     post-drain PING cannot be answered until all older stream bytes reach the
//     server. Application publishing resumes and the false 90s disconnect is
//     removed. An explicitly armed probe still requires its exact message ID.

// v3.9.172 TARGETED LWIP BLYNK-PCB RESET (NO CPU RESET)
//   - v3.9.171 proved that WiFi.mode(WIFI_OFF) is not equivalent to the CPU
//     reset which restores this unit: Arduino-ESP32 already stops/deinitializes
//     esp_wifi and destroys/recreates the STA esp-netif, but the process-wide
//     lwIP TCP/IP core remains alive. Espressif documents esp_netif_deinit() as
//     unsupported at runtime.
//   - The failing run reached 18 Blynk TCP connects. This ESP32 core permits 16
//     active TCP PCBs, uses a 60-second TCP MSL, and its ordinary socket close
//     can leave failed sessions in FIN/retransmit/TIME_WAIT state. A CPU reset
//     clears those PCBs; a WiFi driver restart does not.
//   - On a confirmed Blynk session loss, synchronously ask the lwIP thread to
//     abort ONLY active/TIME_WAIT PCBs whose remote port is BLYNK_PORT. This is
//     the smallest reset domain matching the evidence: other TCP services,
//     sensors and the CPUs are untouched. The same cleanup runs immediately
//     before the rare full WiFi-driver recovery.

// v3.9.171 ACTUAL TCP-ATTEMPT COUNTING + FULL WIFI-DRIVER RESTART
//   - v3.9.170 correctly identified the poisoned ESP32 TCP path, but counted
//     disconnected Blynk.run() passes rather than calls to connect(). This
//     could report four failures after only one real TCP SYN attempt.
//   - Instrument the actual Blynk WiFiClient connect overloads and escalate
//     only after four genuine failed socket connects following an online
//     session.
//   - The larger v3.9.170 log proved that disconnect(false,false) is not
//     sufficient: the first soft STA recovery generated 27 WiFi events and
//     needed ~102s; the second never recovered and the existing software
//     restart immediately restored service. The rare escalation now performs
//     a full ESP32 WiFi-driver stop/start (radio off, credentials retained),
//     then uses the existing non-blocking two-second settle/reconnect path.
//     It does not restart the MCU and cannot affect an ESP8266 build.

// v3.9.170 ESP8266 HEARTBEAT PARITY + POISONED ESP32 TCP-PATH RECOVERY
//   - The working SensorValue6 ESP8266 uses Blynk 0.6.1 defaults: a 10s
//     heartbeat and a 3000ms protocol timeout. Match those values here so the
//     ESP32 and ESP8266 exercise the server with the same Blynk cadence.
//   - v3.9.169 field evidence showed 71s with no inbound Blynk response while
//     WiFi stayed associated at -54..-56dBm, followed by repeated new TCP
//     connect timeouts. This is below Blynk: WL_CONNECTED survived while the
//     ESP32 STA/lwIP data path could no longer establish a fresh TCP stream.
//   - Restores the previously designed but orphaned ESP32-only escalation:
//     after four failed real Blynk TCP reconnects following a proven online
//     session, close Blynk and perform ONE soft STA reassociation
//     (disconnect(false,false), preserving credentials and radio state).
//     Normal Blynk failures still never tear down WiFi, and the escalation is
//     re-armed only after a successful Blynk session. ESP8266 is unaffected.

// v3.9.169 ESP32-ONLY TCP/WIFI RELIABILITY
//   - v3.9.168 disproved ARP expiry as the primary cause: a successful ARP
//     refresh with a cached server entry was observed immediately before the
//     TCP socket still lost ACK progress. The periodic ARP service is therefore
//     no longer invoked.
//   - Disable ESP32 modem sleep for this reliability build. Earlier tests were
//     made before the current single-owner/core-separated protocol path, so
//     they did not isolate power-save behavior from the former reconnect code.
//   - Apply TCP_NODELAY and ESP-IDF TCP keepalive options to the actual Blynk
//     socket at every connection edge. PING is sent immediately rather than
//     waiting behind Nagle, while keepalive supplies bounded dead-peer state.
//   - The installed BlynkProtocol patch now requires BLYNK_TRANSPORT_AWARE.
//     Only this ESP32 sketch defines it; ESP8266 follows untouched v0.6.1 code.

// v3.9.168 LOCAL-SERVER ARP REFRESH
//   - The v3.9.167 field log produced the first repeatable boundary: the live
//     Blynk TCP path stopped at sessionAge=306502ms while WiFi remained
//     WL_CONNECTED. Espressif's bundled lwIP expires dynamic ARP entries at
//     ARP_MAXAGE=300 seconds. BLYNK_SERVER (192.168.3.4) is on this ESP32's
//     own subnet, so every TCP segment depends directly on that ARP mapping.
//   - Refresh the Blynk server's ARP mapping every 60 seconds, safely through
//     tcpip_try_callback(). etharp_request() therefore executes in lwIP's own
//     TCP/IP thread instead of being called unsafely from the Arduino task.
//   - This is preventive maintenance of the one required local next-hop
//     mapping. It does not create another TCP connection, alter Blynk frames,
//     disconnect WiFi, or involve the sensor core.

// v3.9.167 BLYNK PROTOCOL/TRANSPORT COORDINATION
//   - The installed BlynkProtocol.h is patched (with timestamped originals
//     retained beside it) so a locally deferred TCP write is no longer treated
//     like an idle, fully-drained connection by the heartbeat state machine.
//   - While ordered bytes are pending, the protocol services that queue but
//     does not append more heartbeat frames. After drain it sends one PING;
//     the matching server RESPONSE proves all earlier TCP bytes arrived.
//   - The sketch's second, explicit heartbeat sender is disabled. There is now
//     exactly one PING owner: BlynkProtocol::run().
//   - Fragmented 1..4-byte protocol headers are left unread until complete,
//     avoiding a needless blocking read timeout in Blynk.run().

// v3.9.166 MODEM-SLEEP A/B TEST
//   - Enables normal ESP32 minimum-modem WiFi power saving for the controlled
//     comparison prompted by reports that some ESP32/AP combinations fail to
//     reconnect correctly when power saving is forced off, even though an
//     ESP8266 on the same AP remains stable.
//   - Fixes the old contradictory configuration: WIFI_SLEEP could be true,
//     but wifiMarkConnected() unconditionally forced WIFI_PS_NONE afterward.
//     The post-association setting and verification now follow WIFI_SLEEP, so
//     the serial log proves which driver mode is actually active.

// v3.9.165 GROUP-KEY/DATA-PATH FAILURE RECOVERY
//   - The v3.9.164 field log finally supplied a WiFi-layer cause: after the
//     ESP32 uplink stopped, the deferred TCP queue stayed blocked beyond the
//     full 186s Blynk deadline, new TCP/login attempts failed for minutes, and
//     the driver ultimately reported reason 16 (802.11 group-key handshake
//     timeout). WL_CONNECTED had been stale; this was not a Blynk-server loss.
//   - Preserve the live session through normal TCP retransmission first. If a
//     tagged recovery PING still has no RESPONSE after 90s, the encrypted STA
//     data path is considered poisoned and is rebuilt once instead of waiting
//     for the much later WiFi event and guaranteed heartbeat disconnect.
//   - Driver auto-reconnect is enabled for spontaneous AP/security failures.
//     Intentional teardown disables it first, so there are still not two
//     recovery owners racing. The sketch waits 10s for natural reassociation,
//     then performs its bounded fallback.
//   - WiFi teardown/STA restart settle time is 2s. Association timeouts use a
//     soft STA retry, avoiding the repeated RADIO_OFF loop seen in this log.

// v3.9.164 END-TO-END TCP RECOVERY CONFIRMATION
//   - The v3.9.163 field log proved that an empty local deferred-TX queue is
//     not evidence that the server received those bytes: it only means lwIP
//     accepted them. Application publishing resumed too early, then the stock
//     Blynk heartbeat expired while TCP was still retransmitting the missing
//     ESP32->AP/server segment.
//   - A backpressure event now latches an end-to-end recovery gate. V-pin
//     telemetry remains paused after the local queue drains. The next explicit
//     10s Blynk PING is tagged as the recovery probe, and only its matching
//     server RESPONSE releases the gate. This verifies the complete ESP32 ->
//     AP -> server -> ESP32 path rather than a local socket-buffer condition.
//   - Heartbeat is 180s (effective stock deadline 186s). Captured uplink loss
//     took about 58s merely to become locally writable again; 60s left almost
//     no time for the queued probe to cross the repaired TCP stream and return.
//     A genuinely dead session is still closed by the stock protocol deadline.
//   - Removes the ineffective WiFi statistics dump: the precompiled ESP-IDF
//     library has INFO logging compiled out, so result=0 produced no counters.

// v3.9.163 ESP32/AP UPLINK-LOSS ATTRIBUTION + POWER-SAVE INVARIANT
//   - The server capture contains real TCP sequence holes: the server ACK stays
//     at the missing byte while later ESP32 bytes arrive, proving this is not a
//     tcpdump omission or blocked Blynk server handler. ESP-IDF's documented
//     3s, then exponential TCP retransmission timing explains the later repair.
//   - WIFI_PS_NONE is now re-applied and verified after every association, not
//     merely requested before STA startup. This removes driver restart/default
//     restoration as a possible source of an associated-but-sleeping uplink.
//   - A deferred-TX event requests an ESP-IDF WIFI_STATIS_RXTX/BUFFER dump after
//     the ordered backlog drains. The dump is deliberately postponed so debug
//     printing cannot delay the Blynk recovery path. These driver counters let
//     the next field event distinguish ESP32 MAC retries/buffer pressure from
//     loss after the AP has accepted the 802.11 frame.

// v3.9.162 TCP BACKPRESSURE FIX — PRESERVE THE LIVE BLYNK SESSION
//   - The server-side packet capture proves the server continues advertising
//     an open receive window and ACKs every segment it actually receives. In
//     the failing trace, one ESP32->server TCP segment was missing for about
//     47 seconds and later arrived by normal TCP retransmission. This is packet
//     loss below Blynk, not a server quota/idle close.
//   - v3.9.148's 750ms bounded write prevented a long loop stall, but it also
//     called stop() after only 750ms of lwIP send-buffer backpressure. It thus
//     converted a TCP-recoverable loss episode into a guaranteed Blynk loss.
//   - The stock protocol now writes into a small ordered deferred-TX buffer
//     whenever lwIP cannot immediately accept the complete frame. The Core-0
//     loop drains it without blocking, keeps servicing Blynk RX/heartbeat, and
//     pauses new application telemetry until the deferred bytes are accepted.
//     No successful/queued frame is discarded and WiFi is never restarted.
//   - Heartbeat is extended to 60s. The explicit 10s PING remains, while the
//     longer protocol deadline allows TCP retransmission to repair the measured
//     multi-second loss without the protocol destroying an otherwise live
//     connection first. The server learns the new heartbeat in sendInfo().

// v3.9.161 SINGLE-SOCKET BLYNK RECOVERY; WIFI LEFT ALONE
//   - Corrects v3.9.160: INVALID_PMKID and the subsequent authentication
//     failures were produced after firmware-initiated radio teardown and are
//     not evidence that WiFi caused the original Blynk socket loss.
//   - A Blynk write/login failure now resets/retries only the Blynk protocol
//     and its one persistent WiFiClient. It never disconnects or restarts STA.
//   - Removes the separate TCP preflight connection from Blynk reconnects.
//     BoundedWriteWiFiClient now also bounds its own connect() to 1000ms, so
//     Blynk.run() can retry its real socket directly. This avoids probe socket
//     churn, TIME_WAIT entries, and a second connection whose success says
//     nothing about the actual Blynk session.
//   - Restores passive WiFi association handling. A startup attempt timeout is
//     reported as a retry, not a permanent WiFi failure.

// v3.9.160 WIFI DATA-PLANE / PMKID RECOVERY FIX
//   - The v3.9.159 field log gives a repeatable sequence: a 21-byte TCP write
//     cannot enter the send buffer for 750ms, new local-LAN TCP connections
//     subsequently fail while WL_CONNECTED and RSSI=-54..-55 remain, and the
//     existing soft reassociation ends with WiFi reason 49. In ESP-IDF 5.x,
//     reason 49 is WIFI_REASON_INVALID_PMKID. This is below Blynk: the STA may
//     still look associated while its encrypted data path/security state is
//     unusable.
//   - BLYNK_RX_PATH_STALL now stops/restarts the complete STA interface instead
//     of retaining the poisoned driver/PMK state with disconnect(false,false).
//     Credentials are retained and WiFi.begin() is issued only after a 2s
//     non-blocking settle.
//   - WiFi.disconnect() now receives a bounded completion timeout. This removes
//     the observed race where WiFi.begin() tried to set configuration while
//     the driver was still connecting/tearing down.
//   - A disconnect event belonging to the current association attempt is acted
//     on immediately instead of waiting for the misleading 20s timeout. The
//     user message now says the attempt is being retried, not that WiFi is
//     permanently offline.
//   - Server/library audit: the loss at 13.7s cannot be the server's ~69s
//     hardware idle timeout or the 100-msg/s quota; the station is paced at
//     <=5 application writes/s. Blynk's write failure is the detector, not the
//     origin of the failed WiFi/TCP data path.

// v3.9.159 ABSENT ANALOG-SENSOR FIX / v3.9.158 CORRECTION
//   - The MQ-7 and GP2Y1010 are not installed on this station. Their ADC pins
//     are therefore electrically floating; a floating ESP32 ADC can return
//     almost any value and cannot be used as a reliable presence detector.
//   - Explicit feature switches now disable both sensors. No CO PWM is
//     attached or driven, no CO/dust ADC samples are interpreted, and their
//     snapshot values remain SENSOR_UNAVAILABLE.
//   - Withdraws v3.9.158's MQ-7 polarity diagnosis. The logged 5V was a
//     floating-input conversion, not a physical heater measurement. Restore
//     the original active-HIGH control for future use with the documented NPN
//     stage; installation must be enabled explicitly after wiring is verified.
//   - The ESP32 brownout report remains valid, but an absent MQ-7 cannot have
//     caused it. Brownout/power integrity and the WiFi RX-path stall are kept
//     as separate findings rather than claiming an unsupported causal link.

// v3.9.157 PROVEN ONE-WAY WIFI RX-PATH RECOVERY
//   - Correlated the ESP32 log with the live Raspberry Pi TCP state. During
//     the failure the Pi received the ESP32's TCP/Blynk login bytes, but Pi
//     replies and FIN packets remained unacknowledged. Other Blynk clients on
//     the same listener remained healthy, the JVM did not pause/restart, and
//     the Pi's ARP entry matched this ESP32's MAC. This proves the failure is
//     an asymmetric ESP32/AP receive-path stall, not a sensor-core delay and
//     not a Blynk-server outage. WL_CONNECTED is therefore insufficient here.
//   - After four consecutive non-blocking TCP preflight failures following a
//     previously-online Blynk session, perform one SOFT STA re-association.
//     WiFi.disconnect(false,false) keeps the radio/credentials loaded, waits
//     2s for the asynchronous driver teardown, then the existing WiFi state
//     machine calls WiFi.begin(). No ESP restart or radio power-off is used.
//   - Ordinary initial/server outages remain Blynk-only. The STA recovery is
//     limited to one attempt per Blynk outage and is re-armed only after a
//     successful Blynk login, preventing reconnect churn.
//   - Normal 802.11b/g/n (including HT/A-MPDU) remains enabled.

// v3.9.156 NON-BLOCKING STOCK-BLYNK RECONNECT GATE
//   - A disconnected stock Blynk.run() was measured blocking Core 0 for
//     3004ms on every failed TCP connection attempt. Before allowing that
//     path, the existing raw non-blocking probe must now prove that port 8084
//     is accepting TCP connections.
//   - Probe polling always uses select() with a zero timeout. A failed probe
//     backs off Blynk only; it never disconnects an associated WiFi station.
//   - Once the stock WiFiClient has connected and sent HW_LOGIN, subsequent
//     Blynk.run() calls bypass the probe so the same handshake can complete.
//   - Removes the escalation that converted a Blynk-only outage into the
//     observed WiFi status=6/reason=2 offline cascade. Genuine WiFi events
//     remain owned by wifiService().

// v3.9.155 WATCHDOG OWNERSHIP HARDENING
//   - The task watchdog now monitors the explicitly subscribed communications
//     loop and sensorTask only. It no longer subscribes both FreeRTOS idle
//     tasks, whose scheduling can legitimately be delayed by WiFi/lwIP or a
//     long sensor-driver operation even while both application tasks progress.
//   - This removes an avoidable TG0WDT false-reset source without disabling
//     watchdog recovery for a genuinely wedged communications or sensor task.
//   - V61/V62 remain Core-1-sealed immutable pairs. Their embedded keys are
//     authoritative; consumers must commit a joined record only after both
//     keys match because two Blynk virtual-pin widgets are not transactional.

// v3.9.154 SAFE PACING FOR THE SEALED V61/V62 PAIR
//   - Field output proved that Core-1 sealing works: both halves carried the
//     same pair sequence and the same gpsUnix/deviceMs key.
//   - The first deployment also reset while V62 was being issued immediately
//     after V61. Restore the established 200ms transport pacing between the
//     two Blynk frames; this avoids a new back-to-back WiFi/Serial burst.
//   - V62 remains forced ahead of every other application message after V61,
//     and the pair remains immutable. The pacing changes arrival time only;
//     it cannot change or mix the database record key.

// v3.9.153 BOOT-PANIC CORRECTION
//   - Fixes an accidental recursive call in the Core-0 connection snapshot
//     publisher introduced in v3.9.152. The recursion exhausted the loop-task
//     stack at about 0.5s uptime and caused a Core-0 double exception.
//   - The v3.9.152 Core-1-sealed V61/V62 record-pair design is retained.

// v3.9.152 CORE-1-SEALED V61/V62 RECORD PAIRS
//   - Core 1 now serializes V61 and V62 from the same completed SensorData
//     value. One captured gpsUnix/deviceMs key is written into both payloads
//     before either payload crosses to Core 0.
//   - The two immutable strings cross through one one-slot overwrite queue;
//     Core 0 can accept or discard only the complete pair, never independently
//     coalesce V61 from one sensor cycle with V62 from another.
//   - After V61 is written, V62 is forced to be the next application write.
//     The normal transport pacing remains in force so the two protocol frames
//     do not create a new back-to-back WiFi/Serial burst.
//   - WiFi RSSI/quality remain owned by Core 0 and return to Core 1 through a
//     separate one-slot, zero-wait connection snapshot. No sensor driver and
//     no SensorData object crosses in the Core0 -> Core1 direction.
// v3.9.117 reliability hardening:
//   - BLYNK_MSG_LIMIT=0: removes Blynk 0.6.1 nested run()/wait loop; the sketch
//     scheduler is now the only application-TX pacing layer.
//   - BLYNK_HEARTBEAT=20s plus explicit 10s protocol PING so telemetry traffic
//     cannot suppress Blynk liveness checks. A 23s library heartbeat deadline
//     replaces the previous 93s stale-session window.
//   - Non-blocking independent TCP preflight before application TX. This does
//     not replace BlynkSimpleEsp32/WiFiClient; it only prevents entering stock
//     WiFiClient::write() when the Blynk endpoint is plainly unreachable.
//   - A returned TX write >=750ms immediately closes/reconfigures ONLY Blynk;
//     WiFi association is untouched.
//   - Cross-core dataMutex/LEDC lock acquisition is explicitly bounded and
//     instrumented; LEDC lock default wait reduced to 10ms.

// v3.9.111 FINAL FORENSIC EXPERIMENT — diagnostic-only, no transport/recovery change.
//   - Keeps stock BlynkSimpleEsp32/WiFiClient and the exact v3.9.110 test load.
//   - Keeps V61 at one 48-byte changing record/sec: 000001..000009, then repeat.
//   - Adds response-header continuity diagnostics: first-seen timestamp,
//     inter-response gap, same-msgId repeat count, last/new msgId, and the V61
//     sequence visible at the observation point.
//   - Logs both BEFORE and AFTER Blynk.run() peeks. A repeated identical header
//     across both phases is explicitly identified as the same pending bytes; it
//     is NOT counted as a second server response.
//   - Logs the last response header observed before a long Blynk.run()/TX stall.
//   - No new PING, disconnect, reconnect, watchdog, WiFi, timer, or TX policy.
//   - This is the final evidence build before changing the transport/recovery
//     architecture.

// v3.9.108 RX/PONG + WIFI FORENSIC FOLLOW-UP
//   - Keeps the prior V61 forensic/watchdog containment, but changes V61 to the 000001..000009 complex-payload challenge.
//   - Adds STOCK-session RX/PONG-silence milestones at 10s and 70s.
//     IMPORTANT: stock Blynk 0.6.1 does not expose its internal PONG/RESPONSE
//     timestamp to the sketch. These are explicitly RESPONSE-SILENCE OBSERVATION
//     measurements: session age plus heartbeat-loss correlation, not direct
//     packet-level PONG timestamps.
//   - Adds correlation counters for long Blynk.run() calls while the proxy is
//     stale, testing whether a blocking stock socket operation can monopolise
//     Core 1 and delay Blynk protocol heartbeat RESPONSE/PONG processing.
//   - Records when a long Blynk.run() crosses the 10s/70s silence milestones.
//   - Extends unexpected WiFi-disconnect observation from 8s to 30s. During
//     this window the firmware does NOT call WiFi.disconnect(), WiFi.begin(),
//     or Blynk teardown, preserving the original event for attribution.
//   - Adds periodic WiFi-observation snapshots with event counters, Blynk
//     session state, PONG-silence proxy age and Blynk.run blocking statistics.

// v3.9.103 FORENSIC BLYNK LOCK
// This build intentionally removes sketch-initiated escalation while the
// Blynk session is being isolated:
//   * no automatic esp_restart() from health/WiFi timers
//   * no sketch-initiated Blynk.disconnect() during WiFi recovery
//   * no application virtualWrite()/BlynkTimer traffic in CONNECTION_MODE 1
//   * after a stock Blynk session loss, wait 60s before allowing another
//     Blynk.run() connection attempt, so reconnect churn cannot mask the
//     original disconnect event
// Blynk.run() remains the only active Blynk protocol operation.
#define BLYNK_FORENSIC_LOCK 0
// v3.9.110: V61 challenge payload + passive five-byte stock RESPONSE-header peek.
#define BLYNK_FORENSIC_RETRY_HOLDOFF_MS 5000UL

// v3.9.104 V61-ONLY FORENSIC TEST
//   - Keeps the v3.9.103 no-auto-reset/no-Blynk-teardown containment.
//   - Re-enables exactly ONE application datastream: V61, once per second.
//   - V62, V4, legacy dashboard pins, diagnostics and map writes are disabled.
//   - Blynk.run() remains the sole stock session owner. This isolates whether a
//     single authoritative V61 virtualWrite is sufficient to reproduce the
//     10-second NetworkClient stall or subsequent session loss.

// v3.9.101 STOCK BLYNK SINGLE-OWNER CONNECTION MODEL
//   - Removes the sketch-level Blynk.disconnect()/Blynk.config() recovery loop
//     from the active STOCK branch. The stock Blynk protocol is now the sole
//     owner of the Blynk session and Blynk.run() is the only routine used to
//     maintain/re-establish it after a disconnect.
//   - A slow stock WiFiClient::write() no longer triggers an immediate manual
//     session reset. Instead it opens a TX quarantine: all application writes
//     stop, while Blynk.run() continues to service the stock protocol.
//   - After reconnect, telemetry is held for a stabilization window before
//     timers/TX are resumed. This prevents a fresh connection from immediately
//     being hit by the same virtualWrite() workload that caused the stall.
//   - WiFi recovery and final ESP32 reset logic remain independent and are not
//     triggered by a Blynk-only TX stall.

// v3.9.100 changes:
//   - V61/V62 are now a locked coherent pair. Once a snapshot is queued, its
//     exact gpsUnix + deviceMs values are preserved until BOTH V61 and V62 have
//     been delivered or the session is abandoned. The generic per-V-pin queue
//     previously coalesced V61 and V62 independently, allowing V61 from one
//     snapshot to be transmitted with V62 from the next snapshot.
//   - Any Blynk session recovery abandons the entire in-flight V61/V62 pair;
//     the next connected session creates a completely new pair. This prevents
//     ambiguous partial pairs after a socket stall/disconnect.
//   - Failed stock Blynk connection attempts now use a 5s post-attempt backoff
//     measured from the END of Blynk.run(), not 750ms from its start. This stops
//     the observed 3s Blynk.run() login timeout from immediately retriggering
//     another connection attempt every few seconds and starving telemetry.
//   - Successful sessions keep the existing behavior: Blynk.run() continues
//     normally and telemetry is serviced; the backoff applies only while
//     disconnected/connecting.

// v3.9.99 changes:
//   - Stock-Blynk recovery no longer calls Blynk.disconnect() when the stock
//     library has already reported the session disconnected. The v3.9.98
//     field log showed the library printing its Disconnected banner and then
//     never reaching a fresh Blynk SIMPLE connection. Calling disconnect()
//     again on an already-dead stock session can leave the recovery path stuck.
//     Recovery now conditionally disconnects only if Blynk.connected() is still
//     true, then always reapplies Blynk.config() and schedules a fresh run.
//   - Recovery logging brackets the optional disconnect so a future field log
//     can distinguish a stuck Blynk.disconnect() from a failed reconnect.
//   - V10 is now an actual relative IMU Z-axis yaw: BMI160 gyro-Z is integrated
//     continuously, with the existing ZUPT gyro-bias estimate subtracted. The
//     previous V10 mirrored INAV/EKF heading, which stays exactly 0 until a
//     GPS-origin/heading exists; that made V10 remain 0.0 while the unit was
//     being rotated without GPS. V8/V9 remain accelerometer+gyro roll/pitch.
//     V61 yaw uses the same V10/IMU yaw value for coherent telemetry.

// v3.9.96 WIFI-SAFE FOLLOW-UP
//   - v3.9.95 did not change the WiFi connect code, but the supplied field log
//     shows the device never reached Blynk: the STA association itself timed
//     out with disconnect reason 201, followed by our own hard WiFi teardown.
//   - On the FIRST WiFi association only, a 20s connect timeout now performs a
//     soft retry (WiFi.disconnect(false,false) + normal begin after settle)
//     instead of powering the radio off with WiFi.disconnect(true,false).
//   - Established-session WiFi recovery remains unchanged. Blynk-only recovery
//     still never tears down WiFi.

// v3.9.92 CURRENT-BUILD FIXES:
//   1) Stock Blynk transport remains active (BlynkSimpleEsp32/WiFiClient).
//   2) Removed the extra application-level 10s PING from the stock loop; the
//      stock Blynk protocol heartbeat is left to Blynk.run().
//   3) V4 reduced to 5s while V61/V62 remain 1s coherent-record generation.
//   4) Restored all legacy dashboard V-pins V1-V32 that were actually defined
//      by this firmware; v3.9.91 had omitted V15,V16,V17,V19,V26,V30,V31,V32.
//   5) Added V63-V65 TX diagnostics.
//   6) On a long TX stall, purge stale low-priority backlog and close only the
//      Blynk session after the blocked write returns, so reconnect starts clean.
//      This does NOT claim to interrupt the stock WiFiClient::write() itself.

// ============================================================================
// v3.9.67 BLYNK RAW TCP RX-STATE DIAGNOSTICS — diagnostic-only follow-up to v3.9.66
//   - Based directly on v3.9.66. No recovery behaviour is changed.
//   - When the existing passive RX-stall diagnostic fires, additionally inspect
//     the live raw TCP socket without consuming any application data.
//   - Captures select() readability/writability/exception state, MSG_PEEK
//     result/errno, FIONREAD queued RX bytes, SO_ERROR, SO_RCVBUF/SO_SNDBUF,
//     SO_KEEPALIVE, socket fd, connection age, and local/remote ports.
//   - This is intended to distinguish: (A) healthy TCP socket with no bytes
//     arriving from the server, (B) data queued but not being consumed,
//     (C) TCP/socket error/FIN, or (D) an internally abnormal socket state.
//   - MSG_PEEK is used deliberately so this diagnostic never consumes Blynk
//     protocol bytes. select() uses a zero timeout, so it does not add a
//     blocking wait to loop().
//   - NO recovery thresholds, disconnect calls, timers, WiFi settings, or
//     Blynk state transitions are changed by this diagnostic.
//
// v3.9.66 BLYNK RX-STALL DIAGNOSTICS — diagnostic-only follow-up to v3.9.65
//   - Based directly on v3.9.65 WiFi teardown attribution build.
//   - Adds passive [Blynk RX STALL] logging once RX silence reaches 10s, then
//     every 2s while the Blynk session still reports connected.
//   - Captures RX age, TX backlog/ages, Blynk.run() maximum/slow count, raw
//     transport read/write activity, EAGAIN/timeouts/socket errors, errno/SOERR,
//     handshake/probe state, and heap.
//   - Uses Serial.printf() so DEBUGON=OFF cannot hide this diagnostic.
//   - NO recovery thresholds, disconnect calls, timers, WiFi settings, or
//     Blynk state transitions are changed by this diagnostic.
//
// v3.9.62 WIFI EVENT ATTRIBUTION DIAGNOSTICS + GPS LOG SILENCE
//   - Adds explicit attribution for STA_DISCONNECTED events: expected when
//     generated by our own WiFi.disconnect(), versus unexpected when reported
//     without an intentional teardown in progress.
//   - Logs WiFi event sequence, event ID, reason, event age, RSSI, channel,
//     BSSID, IP and gateway when the reconnect state machine reacts.
//   - Logs GOT_IP details after a successful association.
//   - Adds the same event counters to the WiFi failure snapshot immediately
//     before any WIFI_FAILURE_LIMIT reset.
//   - No recovery thresholds or connection behaviour are changed. This build
//     is diagnostic-only for the WiFi trigger question.
//   - GPS_DEBUG_PRINTOUT is disabled to remove verbose GPS diagnostic output.
//
// v3.9.59 CORE 3.3.11 SERVER-SIDE + RESET-PATH DIAGNOSTICS — COMPILE FIX
//   BASELINE: v3.9.51 — latest known-good Arduino-ESP32 core 2.0.17 build.
//   - This build intentionally starts from v3.9.51 and changes only APIs required
//     by Arduino-ESP32 3.3.x, plus the small Core-1 diagnostics below.
//   - WiFi configuration, TX power (15dBm), sleep setting, reconnect ownership,
//     Blynk timeouts/budgets, telemetry cadence, task affinity/priorities, sensor
//     scheduling, GPS, INAV, I2C recovery, and application logic are preserved.
//   - LEDC: replaces the removed 2.x channel-first API with the 3.x pin-based API,
//     while retaining the existing explicit CO/buzzer channel assignments.
//   - I2S: replaces the legacy driver/i2s.h microphone implementation with the
//     ESP-IDF 5.x standard driver/i2s_std.h implementation; microphone settings
//     and processing behavior are preserved.
//   - Task WDT: replaces the 2.x esp_task_wdt_init(10,true) call with the ESP-IDF
//     5.x config/reconfigure API, preserving the 10-second + panic intent.
//   - Adds a compile-time guard for Arduino-ESP32 3.x.
//   - Diagnostic-only additions: Blynk.run duration guard/counters and one FreeRTOS
//     tick at the end of loop() to give CPU1 IDLE1 an explicit scheduling point.
//   - Blynk socket timers remain exactly as in v3.9.51: 3000ms read timeout and
//     750ms write budget. No timeout reduction is included.
//   - Adds diagnostic-only reset-path identification before every firmware restart.
//   - Adds passive server-health counters: TCP probe/connect timing, Blynk handshake
//     duration, RX age, TX queue depth, TX success/failure/timeout counters, socket
//     age, last socket error, and Blynk.run timing. No new recovery action is added.
// ============================================================================

// v3.9.50 ATGM336H/AT6558 GPS CONFIGURATION FIX — USE PCAS, NOT PMTK
//   Field testing proved that this ATGM336H receiver does not respond to the
//   PMTK251 baud command/PMTK001 ACK scheme used by the previous firmware.
//   The receiver's documented AT6558/ATGM336H command family is PCAS instead:
//     PCAS01 = UART baud rate
//     PCAS02 = update/fix interval
//     PCAS03 = NMEA sentence selection
//     PCAS00 = save configuration to flash
//   FIX: initGPS() now uses PCAS commands for this receiver. NMEA output is
//   reduced to GGA+RMC before the baud change, PCAS01 selects 115200, actual
//   NMEA reception verifies the new speed, PCAS02 selects 100ms/10Hz, and
//   PCAS00 saves the configuration. GPS+BDS constellation selection is left
//   unchanged. PCAS commands do not use the PMTK001 ACK mechanism, so baud
//   verification is deliberately based on real NMEA bytes at the new speed.
//
//   Checksum note: direct XOR gives PCAS03 GGA+RMC = *02, PCAS01,5 = *19,
//   PCAS02,100 = *1E, PCAS00 = *01. The supplied reference lists PCAS02,100
//   as *1D; that checksum is incorrect for the stated XOR algorithm.
//

// v3.9.51 GPS CONFIGURATION PERSISTENCE POLICY
//   - Only perform GPS reconfiguration and PCAS00 flash-save when auto-detection
//     finds the receiver at 9600 baud. This is the known factory/default state
//     from which we deliberately migrate the receiver to 115200 + 10Hz.
//   - If auto-detection finds ANY baud rate above 9600, the receiver is treated
//     as already configured. The firmware does NOT send PCAS03/PCAS01/PCAS02
//     and, critically, does NOT issue PCAS00 on that boot. It simply keeps the
//     detected working baud and continues with normal GPS operation.
//   - PCAS00 is therefore a one-time persistence operation for the 9600 ->
//     115200 + 10Hz migration, rather than a flash write on every ESP32 boot.
//   - The existing baud ladder is retained as a fallback only if the migration
//     from 9600 cannot verify 115200; PCAS00 is NOT sent after such a fallback.
//
// v3.9.52 ARDUINO-ESP32 CORE 3.3.11 MIGRATION — MINIMAL API-ONLY UPDATE
//   - Keeps all application behaviour, pins, timing, GPS PCAS persistence policy,
//     Blynk transport, WiFi recovery, I2C recovery, INAV, diagnostics, and sensor
//     scheduling unchanged. Only APIs removed/changed by Arduino-ESP32 3.x are
//     translated to their current equivalents.
//   - LEDC: replaces the removed 2.x channel-first API (ledcSetup/ledcAttachPin/
//     channel-first ledcWrite) with the 3.x pin-based API while retaining the
//     explicit channels 0 and 8 so CO PWM and buzzer remain in separate groups.
//   - I2S: uses the ESP-IDF 5.5 standard I2S driver (driver/i2s_std.h) already
//     intended by the v2.2 design notes. The microphone remains 16kHz, 32-bit,
//     stereo capture on GPIO25/26/33 and the existing two-slot RMS selection is
//     preserved. This removes the legacy I2S driver dependency completely.
//   - Task WDT: migrates the old esp_task_wdt_init(10,true) call to the ESP-IDF
//     5.x esp_task_wdt_config_t API and reconfigures an already-initialized TWDT
//     to the same 10-second/panic intent. Sensor-task subscription/feed logic is
//     unchanged.
//   - Adds a compile-time guard so this exact build is not accidentally flashed
//     with an older Arduino-ESP32 core.
// ============================================================================

// v3.9.49 GPS ACK-READ TIMING FIX + CORRECTION OF v3.9.48's CONCLUSION
//   Field result after v3.9.48: still no ACK ever seen, across every rate,
//   even with PMTK314 now sent first. This disproves v3.9.48's theory that
//   heavy default sentence output was preventing the module from replying —
//   if that were the cause, moving PMTK314 first should have helped, and it
//   did not change the outcome at all.
//   CORRECTION: v3.9.48 also misread its own evidence. It treated seeing
//   "$GNGGA,..." lines in the capture as proof the RMC+GGA filter had NOT
//   taken effect. That reasoning was wrong — GGA is one of the two sentence
//   types the filter deliberately KEEPS enabled (see the PMTK314 field
//   mapping: GLL=0 RMC=1 VTG=0 GGA=1 GSA=0 GSV=0 ...). Seeing GGA in the
//   output is expected regardless of whether the filter applied, so it was
//   never actually evidence against the v3.9.48 fix — the reorder itself is
//   left in place (it is still correct per the documented guidance to
//   reduce output before requesting a rate the current config cannot
//   support), it just was not the actual cause of the missing ACK.
//   Re-examined the ACK-read code itself and found two real bugs there
//   instead: (1) it started listening for a reply immediately after
//   flush() with NO settle delay — flush() only guarantees our own bytes
//   finished transmitting, it says nothing about the module's own
//   processing/reply latency, so the listen window could easily elapse (or
//   land mid-capture) before any real ACK was ever sent; (2) it only ever
//   read ONE line before giving up — if the module's ACK was queued behind
//   an already-in-progress regular NMEA sentence (likely, since a module
//   normally finishes what it is sending before starting something new),
//   that unrelated sentence would be captured and the read would stop
//   there, never reaching the actual ACK.
//   Fix: added a 50ms settle delay before listening, and the read now scans
//   up to GPS_ACK_MAX_LINES (6) lines within a 400ms window, specifically
//   matching lines that start with "$PMTK001" (the real ACK prefix for this
//   command family) instead of accepting whatever line arrived first. Non-
//   matching lines are now also logged (as "non-ACK line N, ignoring") so
//   the next boot log will show clearly whether an ACK is present further
//   down the stream, or is genuinely absent entirely.
//
// v3.9.48 GPS COMMAND ORDER FIX — SENTENCE FILTER BEFORE BAUD SWITCH
//   Field evidence from v3.9.47's ACK-response diagnostic: the captured
//   "responses" to PMTK251 were not ACK sentences at all — they were plain
//   GGA/GSV fragments (e.g. "$GNGGA,185249.000,,,,,0,...",
//   "8,,25,63,270,,28,20,304,,0*67", one even binary-looking), on every
//   attempt, at every candidate baud rate (115200 and 57600 failed
//   IDENTICALLY, which rules out a baud-specific electrical/timing limit —
//   a real limit would show lower rates doing better, not failing the same
//   way). This means the module's transmit buffer was so busy with its full
//   DEFAULT NMEA sentence set (not just RMC+GGA — that filter was
//   previously only applied AFTER the baud ladder) that our listen window
//   never caught an actual reply, only regular ongoing traffic.
//   Fix: moved the PMTK314 sentence-filter command (limit output to
//   RMC+GGA only) to run FIRST, immediately after auto-detection, at the
//   confirmed-working detectedBaud — before the baud-switch ladder even
//   starts. This deliberately reverses the "baud before rate/filter"
//   ordering used in v3.9.44-47, which was based on generic guidance from
//   other MTK-family module reports; that guidance assumes a module that
//   reliably receives commands in the first place, which the evidence above
//   suggests was not true here at the module's heavy default output level.
//   Reducing output load BEFORE asking for a baud change gives the module
//   much more room to actually receive and process PMTK251 afterward.
//   Known open question, not yet confirmed either way: some MTK-family
//   modules reset their output pipeline on a baud change (see v3.9.45/46
//   notes), which could in principle revert this sentence filter partway
//   through the ladder. Deliberately NOT adding speculative re-send-before-
//   each-attempt logic for this — the v3.9.47 ACK-read diagnostic is already
//   in place and will show directly on the next boot whether an actual
//   PMTK_ACK response appears now, which will confirm or rule this out with
//   real evidence rather than another guess.
//
// v3.9.47 GPS PMTK251 COMMAND VERIFIED CORRECT + ACK-RESPONSE DIAGNOSTIC
//   Field result after v3.9.46: 115200 failed after 2 attempts, THEN 57600
//   also failed after 2 attempts, in exactly the same pattern. Re-verified
//   every PMTK251 checksum in the ladder (115200/57600/38400/19200) by
//   direct XOR calculation — all four are correct as sent. The command
//   itself is not the problem.
//   Two different rates failing identically rules out a baud-specific
//   electrical/timing limit (a real limit would more likely show lower
//   rates succeeding where higher ones fail). Research turned up a directly
//   relevant real-world report on this exact MTK command family: the module
//   can reply to PMTK251 with an explicit PMTK_ACK sentence —
//   $PMTK001,251,3*36 (succeeded) or $PMTK001,251,2*37 ("valid packet, but
//   action failed") — and that a rejection can happen if the requested baud
//   cannot support the module's CURRENTLY CONFIGURED update rate/sentence
//   list. We were never reading this response at all; failure/success was
//   only ever inferred indirectly from whether NMEA reappeared afterward,
//   which cannot distinguish "module explicitly rejected the command" from
//   "module never received it" from "module accepted it but needs more
//   settle time than we gave it."
//   Fix (diagnostic only, no behavior change yet): read and print whatever
//   the module sends back within 300ms of the PMTK251 command, BEFORE
//   switching the ESP32's own UART away from the rate the command was sent
//   at. This will show the module's actual PMTK_ACK response (or its
//   absence) directly in the boot log on the next run, which turns "why does
//   it fail" from inference into direct evidence — the next log capture
//   will show definitively whether this is an explicit rejection, a timeout
//   with no response, or something else, and the fix can then target the
//   real cause instead of another guess.
//   NOTE: did NOT reorder the existing PMTK251 (baud) -> PMTK220 (rate) ->
//   PMTK314 (sentence filter) sequence — both independent sources found
//   during this investigation agree baud should be changed first, which
//   this file already does.
//
// v3.9.46 GPS BAUD LADDER + RATE-MATCHED UPDATE FREQUENCY
//   Two related fixes on top of v3.9.45:
//   1. Previously only 115200 was ever attempted before falling all the way
//      back to the auto-detected rate (often 9600) on failure. Replaced
//      with a proper ladder: 115200 -> 57600 -> 38400 -> 19200, each rung
//      tried GPS_BAUD_LADDER_ATTEMPTS (2) times before moving to the next
//      rate down, only falling back to the confirmed-working detectedBaud
//      if every rung fails. This gives a much better chance of landing on
//      SOME higher rate than 9600 even when 115200 specifically does not
//      work on a given unit/wiring.
//   2. 9600 baud structurally CANNOT sustain 10Hz RMC+GGA NMEA output: at
//      8N1 framing, 9600 baud provides ~9600 bits/s of capacity, while
//      RMC+GGA at 10Hz needs roughly 14500 bits/s (calculated from typical
//      sentence lengths) — a 0.66x margin, i.e. it cannot keep up even in
//      principle. This matches MediaTek's own PMTK documentation, which
//      explicitly warns to reduce NMEA sentence output or lower the update
//      rate if the host baud rate cannot support it. Previously the update
//      rate was unconditionally requested at 10Hz regardless of which baud
//      the device ended up on — meaning a 9600 fallback was being asked to
//      do something it physically cannot do, on top of already being a
//      fallback. Fixed: the update-rate request now follows the settled
//      baud (10Hz for any settled rate >=19200, which all have ample
//      margin; 5Hz only if settled at 9600, which fits with margin). All
//      PMTK251/PMTK220 checksums for every rate in the ladder were
//      calculated and verified directly, not guessed.
//   Status messages (Serial + engMsg) now report the actual settled baud
//   AND actual update rate in use, instead of a value that could be stale
//   after a ladder fallback.
//
// v3.9.45 GPS BAUD-SWITCH RETRY + PMTK314 CHECKSUM FIX
//   Field result after v3.9.44: the single-attempt verification (250ms
//   settle + 1.5s listen) correctly detected the failure and safely
//   reverted to 9600 baud instead of running dark — but the switch to
//   115200 still never succeeded, on every boot.
//   Investigated why: this GPS module's command family (PMTK251, shared by
//   several GlobalTop/MTK-chipset-based modules) is documented in the field
//   by other users of the same command set to sometimes RESET/reboot the
//   module's NMEA output pipeline after a baud-rate change, not just switch
//   UART speed cleanly. A single ~1.5s verification window may not reliably
//   outlast that reset, especially on a cold/warm start.
//   Fix: restructured the single attempt into a retry loop
//   (GPS_BAUD_SWITCH_ATTEMPTS=3). Each attempt: re-send PMTK251 (at
//   whatever baud the module is confirmed listening on), switch the ESP32's
//   UART, then verify for 2.5s (up from 1.5s). The settle delay before
//   switching widens on each retry (250ms x attempt number) in case the
//   module needs more time to come back up after a reset. Only reverts to
//   the confirmed-working detectedBaud after ALL attempts fail — this does
//   not mask a genuine incompatibility (if this specific unit/firmware truly
//   cannot reach 115200, every attempt fails the same way and the revert
//   still happens, just after real attempts instead of one).
//   Also fixed, found while reviewing this code: the PMTK314 (NMEA sentence
//   filter, RMC+GGA only) command's checksum was *28 but the correct XOR
//   checksum for that exact command body is *34 — verified by direct
//   calculation. A module correctly validating checksums would have
//   silently rejected this command every boot, leaving ALL NMEA sentence
//   types enabled instead of just RMC+GGA (wasted UART bandwidth and log
//   clutter, not a correctness/safety issue, since the parser only acts on
//   RMC/GGA regardless of what else is present). PMTK220 (10Hz update rate)
//   and PMTK251 (baud) checksums were independently re-verified and are
//   correct as-is.
//
// v3.9.44 GPS BAUD-SWITCH VERIFICATION FIX
//   Field symptom: PPS was locked (proving the GPS module had a satellite
//   fix and its 1PPS line was toggling) but zero NMEA data was ever parsed
//   (gpsDollarCount/gpsRawLineCount stayed 0 indefinitely while
//   gpsUartBytesReceived climbed steadily — bytes were arriving on the wire
//   but never aligned into a valid '$...' sentence).
//   Root cause: after auto-detecting the module at a working baud rate
//   (e.g. 9600) and sending PMTK251 to ask it to switch to 115200, the
//   firmware waited only 100ms before switching its OWN UART to 115200 and
//   proceeding — with no check that the module had actually applied the
//   change yet. If the module was still transmitting at the old rate when
//   the ESP32 started listening at the new one, every subsequent byte
//   landed misaligned, and nothing in the firmware ever re-checked or
//   recovered — it would silently and permanently run "dark" on GPS for the
//   rest of the session, indistinguishable from a hardware fault without
//   deep log inspection.
//   NOTE: this is unrelated to gps_serial.baudRate() sometimes reading back
//   115201 instead of 115200 — that is a normal ESP32 UART clock-divider
//   rounding artifact (~0.001% off), used only for logging, and far too
//   small on its own to cause framing errors. Left as-is.
//   Fix: gps_serial.flush() before switching (guarantees the PMTK251 command
//   bytes are actually clocked out, not just sitting in the TX buffer),
//   longer settle delay (100ms -> 250ms), and — the actual fix — an active
//   post-switch verification step that listens for 1.5s for a real '$...'
//   NMEA sentence at the new rate. If verification fails, the firmware
//   reverts to the already-confirmed-working detectedBaud instead of
//   silently proceeding blind, so the device still gets usable GPS data
//   this session. The final status message now reports the actual live
//   baud rate in use instead of an unconditional "115200" string, since a
//   revert can make that claim false.
//
// v3.9.43 WIFI/BLYNK CONNECTION RELIABILITY + SERVER-SIDE STALL RECOVERY
//   Multi-session investigation into two related but distinct problems:
//   (A) the device's own WiFi/Blynk reconnect logic, and (B) an intermittent
//   BRIEF socket write stall (errno=11/EAGAIN) caused by the Blynk SERVER
//   (a self-hosted Java process on a Raspberry Pi), not by this firmware or
//   the network. Fixes for (A) are in this file; (B) required a change on
//   the server's systemd unit (blynk.service) — see the note at the very end
//   of this changelog block, since no amount of firmware change can fix a
//   problem on the peer.
//
//   (A) WiFi/Blynk reconnect fixes, all in this file:
//   - WiFi reconnect is now event-driven: the WiFi disconnect callback sets
//     wifiDisconnectEventPending, and wifiService() reacts on the very next
//     loop() pass instead of waiting on the old 30s WIFI_RECONNECT_MS poll.
//     That poll remains only as a fallback in case an event is ever missed.
//   - Every WiFi reconnect now logs the real disconnect reason code and
//     RSSI (via engMsgf, always visible regardless of DEBUGON) instead of a
//     generic "reconnect attempt" message. This is what let us prove that an
//     earlier watchdog (see below) was self-disconnecting a healthy session:
//     reason=8 (ASSOC_LEAVE) is what the driver reports for OUR OWN
//     WiFi.disconnect() call, not an external kick or weak signal.
//   - WiFi TX power backed off from max (WIFI_POWER_19_5dBm) to a moderate
//     WIFI_POWER_15dBm. Max TX power on a nearby AP can itself provoke
//     AP-side instability; this was a plausible contributor, not confirmed
//     needed, but is a safe precaution.
//   - Added, then REMOVED, an RX-staleness watchdog (v3.9.40/41) that forced
//     a reconnect when no bytes had been read for N seconds. It repeatedly
//     tripped on a perfectly healthy, already-connected session (a PONG
//     reply lagging past the threshold was enough), which called
//     Blynk.disconnect()+WiFi.disconnect() on a working connection — this
//     was the actual cause of the "reason=8 every ~3 minutes" churn seen in
//     v3.9.41 field logs, not a real network problem. blynkLastRxMillis
//     itself is kept as a passive diagnostic value only (see [Blynk STATUS]
//     below); nothing acts on it anymore.
//   - The pre-existing probe/handshake-failure escalation ladder (probe
//     timeout -> handshake timeout -> WiFi reset after N handshake fails ->
//     ESP restart after M WiFi-reset fails) was NOT buggy, just slow by
//     construction: field logs showed ~2m38s from a genuine write-timeout to
//     a full software restart (10s probe/handshake/retry timeouts x 3 fails
//     x 3 tiers). Tightened every stage (probe 6s->4s, handshake 10s->6s,
//     retry gate 10s->4s, handshake-fails-before-WiFi-reset 3->2, WiFi-
//     reset-fails-before-ESP-restart 3->2). New worst case ~55-60s; typical
//     case (server responds within one cycle) recovers in well under 15s.
//     The tiered structure itself is kept — a single fast retry is not
//     enough on its own (a reachable TCP port does not guarantee an
//     immediate Blynk login response); see v3.9.9/v3.9.10 notes below for
//     why that matters.
//   - Diagnostics cadence (V34-V60, 22 fields — retained maxima/counters,
//     not live data) slowed from 10s to BLYNK_SEND_DIAG_MS (30s), reducing
//     steady-state write volume to the server. Confirmed by a live test that
//     this alone does not eliminate the server-side stall (see part B) —
//     kept anyway as reasonable load reduction, and paired with:
//   - TX packet pacing slowed from BLYNK_TX_INTERVAL_MS=100ms (<=10 pkt/s)
//     to 150ms (<=~6.7 pkt/s) — smoother, less bursty delivery to the
//     server, same total data (the existing per-vpin coalescing queue is
//     unchanged).
//   - New: a brief post-reconnect TX grace period
//     (BLYNK_TX_POST_RECONNECT_GRACE_MS, 2s). After a FRESH Blynk connection
//     is confirmed, telemetry sending (blynkTxService()) pauses briefly
//     before resuming, so a backed-up queue (46 pending items observed in
//     one field log) does not immediately burst at a server that has just
//     barely recovered. Blynk.run()/heartbeat/handshake are unaffected.
//   - New passive-only [Blynk STATUS] diagnostic line (every
//     BLYNK_STATUS_LOG_MS=5s while connected): connection state, WiFi
//     status/RSSI, RX age, TX queue depth, TX attempt/success age,
//     handshake/force-reconnect flags. Takes no action and never sets
//     blynkForceReconnect or calls disconnect() — see the RX-staleness
//     watchdog history above for exactly why that distinction matters.
//   - Fixed two PRE-EXISTING latent bugs, both only surfaced when DEBUGON
//     was set true for this investigation (DBG_PRINTF/DBG_PRINTLN compile to
//     a no-op when DEBUGON=false, so broken code inside them was previously
//     invisible to the compiler):
//       * Two DBG_PRINTF calls (BMI160 and ENS160/AHT2x I2C-recovery
//         messages) had a raw line break embedded in the string literal
//         instead of \n — fixed.
//       * i2cLastSdaLevel/i2cLastSclLevel were declared ~380 lines after
//         their first use inside readENS()'s diagnostics — moved the
//         declarations earlier, removed the duplicate later copy.
//   - DEBUGON is false again (normal operation). It was set true only for
//     the duration of this field investigation to make the above visible;
//     every behavioral fix above is independent of DEBUGON and stays active
//     either way.
//
//   (B) Confirmed root cause of the actual multi-minute stalls: the Blynk
//   SERVER (Java, self-hosted on a Raspberry Pi via systemd) was
//   occasionally not draining its socket for 750ms+ (errno=11/EAGAIN on our
//   write), most likely tied to G1GC allocation/marking pressure — a GC log
//   on the server showed a 180-second gap between collections followed by a
//   "Prepare Mixed" cycle landing ~4s after one such stall, plus 11
//   "humongous" old-gen regions indicating G1 region sizing was suboptimal.
//   A comparison device (ESP8266, same server, ~9x less traffic, stock
//   BlynkSimpleEsp8266 library) never showed this symptom — but cutting this
//   ESP32's own traffic 3x (the diagnostics-cadence change above) did NOT
//   stop the stalls either, which is what pointed conclusively at the server
//   rather than this device's code or traffic volume.
//   FIX WAS APPLIED ON THE SERVER, NOT HERE — blynk.service (systemd unit)
//   JVM flags changed: -Xmx 500m->700m, -XX:G1HeapRegionSize 3m->4m (3m was
//   not even a valid power-of-two value; region size must be a power of two,
//   and RAISING it — not lowering it — is what reduces humongous-object
//   allocations, since the humongous threshold is exactly half the region
//   size), -XX:InitiatingHeapOccupancyPercent=45 made explicit,
//   -XX:ConcGCThreads=1, -XX:ParallelGCThreads=4 (matches the Pi's 4 cores),
//   -XX:G1ReservePercent=15 (up from default 10, more evacuation headroom).
//   MemoryMax raised 750M->800M to keep headroom above the new -Xmx. Applied
//   and field-confirmed stable: after this change, a Blynk reconnect
//   resumes normal [Blynk TRANSPORT]/[Blynk TX] activity and data updates
//   continue, instead of cascading into the old ~2m38s recovery/restart
//   cycle.
//
// v3.9.39 BLYNK FORCED WIFI-RECOVERY FIX
//   - v3.9.38 correctly fixed clean power-on by avoiding aggressive teardown on
//     the first WiFi association. However, after a real WiFi failure the Blynk
//     protocol object can still report connected even though its TCP session is
//     dead. The v3.9.38 probe gate required !Blynk.connected(), so the recovery
//     could stop at "handshake pending" forever.
//   - v3.9.39 adds blynkForceReconnect. It is set only after a subsequent WiFi
//     re-association, forces a fresh Blynk/socket teardown, and bypasses the
//     stale Blynk.connected() gate until a NEW Blynk connection is confirmed.
//   - Clean power-on behavior is unchanged.
//   - Existing I2C recovery, bounded raw transport, WiFi reconnect state
//     machine, watchdogs, diagnostics, and telemetry logic are preserved.
//
// v3.9.38 BLYNK/WIFI RECOVERY FIX
//   - Restores the clean power-on Blynk startup path from v3.9.34.
//   - v3.9.35-3.9.37 called Blynk.disconnect() during EVERY WiFi association,
//     including the first association after power-on. That teardown is only
//     needed when recovering from a previous WiFi session.
//   - On first WiFi association: configure Blynk and immediately request the
//     bounded TCP probe, without first destroying the fresh Blynk state.
//   - On subsequent WiFi associations: keep the aggressive Blynk/raw-socket
//     teardown so a stale Blynk session cannot survive a WiFi reconnect.
//   - Keep the v3.9.17+ WiFi reconnect state machine and 250ms disconnect
//     settle window; WiFi.setAutoReconnect(false) remains intentional because
//     wifiService() is the single owner of reconnect timing.
//   - Keep the v3.9.37 explicit immediate-probe flag; no millis() subtraction
//     trick is used near boot.
// v3.9.33 compile fix: keep this threshold in the early configuration
// section so Arduino-generated function prototypes can see it.
#define I2C_DIAG_SLOW_MS 100UL

// v3.9.35 I2C SENSOR RECOVERY — fixes shared-bus loss after long runtime
//   - If ENS160/AHT2x loses I2C communication after it was previously healthy,
//     ensReady is now cleared on a failed read so the next recovery attempt can
//     actually rerun initENS(). Previously ensReady could remain true forever,
//     leaving a physically recovered device with stale library state.
//   - If BMI160 has sustained read failures, bmi160Ready is cleared and the
//     sensor is fully re-probed/reinitialized instead of remaining permanently
//     marked ready after the bus/device disappears.
//   - I2C recovery now explicitly releases the bus, performs the 9-clock recovery,
//     reinitializes Wire, and verifies the bus lines before sensor reinit.
//   - Recovery is conservative and rate-limited; no sensor/Blynk scheduling is
//     removed and no external reset pin is assumed.
//   - If the common 3.3V rail or a sensor itself has latched into a power fault,
//     software I2C recovery cannot replace a real power cycle; the firmware logs
//     that condition instead of silently remaining in a stale-ready state.
// ============================================================================

// v3.9.32 DIAGNOSTICS:
//   - Keeps Arduino-ESP32 core 2.0.17 compatibility unchanged.
//   - Preserves BMI160 0x69 as the preferred address, but auto-tests 0x69 and
//     0x68 and requires chip ID 0xD1 before selecting the active address.
//   - Adds a one-time I2C scan for AHT2x/ENS160/BMI160 addresses.
//   - Adds granular timing/error diagnostics around BMI160 and ENS160 I2C calls.
//   - Adds SDA/SCL and Wire error reporting for slow/failing I2C probes.
//   - No sensor scheduling, INAV, Blynk, WiFi, or recovery logic is removed.
// ============================================================================
// v3.9.32 GPS RAW-NMEA CAPTURE — additive diagnostic only
//   - Captures up to 6 complete lines beginning with '$' from UART1 GPIO13.
//   - Prints the exact printable characters plus escaped non-ASCII bytes.
//   - Independently calculates the NMEA XOR checksum and compares it with the
//     transmitted two-digit checksum.
//   - Does not alter the bytes passed to TinyGPS++; does not change GPS baud,
//     PPS, fix logic, INAV, Blynk, WiFi, sensor scheduling, or recovery.
//   - Fixed-size buffers are used; no Arduino String/dynamic allocation.
// ============================================================================

// ============================================================================
// Outdoor Air Quality Station — ESP32 Production Firmware v3.9.32
// Migrated & rewritten from original Arduino Mega + Esp8266EasyIoT (AM, 2017)
//
// ─── HARDWARE v2.0/2.1 ───────────────────────────────────────────────────────
//   KEPT:       MQ-7 CO board (5V heater, closed-loop via A1 feedback)
//               ML8511 UV (3.3V ratiometric, direct)
//               Sharp GP2Y1010 dust (5V, 10k+10k divider)
//               JY-901/WT901 IMU (UART2 GPIO16/17, 3.3V)
//               LEDs, buzzer, CO PWM
//
//   REPLACED:
//     DHT21          → AHT2x on ENS160 board   (I2C 0x38, 3.3V, no pull-up)
//     MICS-2710 NO2  → ENS160 TVOC/eCO2/AQI    (I2C 0x53, 3.3V, no divider)
//     Analogue mic   → INMP441 I2S 24-bit mic   (GPIO25/26/33, 3.3V)
//     u-blox GPS     → ATGM336H NMEA GPS        (UART1 GPIO13/23, 9600 baud)
//
// ─── MQ-7 HEATER BOARD — CLOSED LOOP (from datasheet analysis) ──────────────
//   Board pins:
//     VCC → ESP32 VIN (5V)
//     GND → GND
//     D2  → GPIO4  (PWM direct, 3.3V logic drives NPN base via 1kΩ — sufficient)
//     A0  → 10kΩ+10kΩ divider → GPIO36 (CO sense, smoothed by 470µF on board)
//     A1  → 10kΩ+10kΩ divider → GPIO39 (heater voltage feedback, 0–5V → 0–2.5V)
//
//   New closed-loop calibration: instead of open-loop PWM sweep, we read A1
//   (heater voltage feedback) and adjust duty cycle until A1 hits target voltage.
//   Heating phase:     target A1 = 5.0V  → duty = 255 (full on)
//   Measurement phase: target A1 = 1.4V  → duty adjusted via closed loop
//
// ─── PIN MAP v2.0 ────────────────────────────────────────────────────────────
//   GPIO36 VP — CO sense AO (10k+10k divider from board A0) - Suitable for reading analog signals
//   GPIO39 VN — CO heater feedback A1 (10k+10k divider from board A1) - Suitable for reading analog signals
//   GPIO34    — UV OUT (ML8511, direct 3.3V)
//   GPIO35    — UV REF (ML8511, direct 3.3V)
//   GPIO32    — Dust AO (GP2Y1010, 10k+10k divider)
//   GPIO21    — I2C SDA (ENS160+AHT2x)
//   GPIO22    — I2C SCL (ENS160+AHT2x)
//   GPIO13    — GPS UART1 RX ← ATGM336H TX  [moved from 22 to free I2C SCL]
//   GPIO23    — GPS UART1 TX → ATGM336H RX
//   GPIO16    — [SUPERSEDED v2.4] was JY-901 UART2 RX — now free/unused
//   GPIO17    — [SUPERSEDED v2.4] was JY-901 UART2 TX — now free/unused
//   GPIO25    — INMP441 I2S SCK (BCLK)
//   GPIO26    — INMP441 I2S WS  (LRCLK)  [ADC2 output only — no WiFi ADC conflict]
//   GPIO33    — INMP441 I2S SD  (data in) [freed from NO2 analogue]
//   GPIO4     — CO PWM (LEDC, direct to D2 on heater board)
//   GPIO18    — Dust IR LED (active LOW)
//   GPIO14    — LED green   (CO ≤ 10 ppm)
//   GPIO27    — LED orange  (CO 10–20 ppm)
//   GPIO15    — LED red     (CO > 20 ppm) [GPIO26 freed for I2S WS]
//   GPIO19    — Buzzer
//
// ─── VOLTAGE DIVIDERS v2.0 ───────────────────────────────────────────────────
//   Only 3 remain (all new sensors are 3.3V native):
//   MQ-7  A0 → 10kΩ + 10kΩ → GPIO36    Vout@5V = 2.5V ✓
//   MQ-7  A1 → 10kΩ + 10kΩ → GPIO39    Vout@5V = 2.5V ✓ (heater feedback)
//   GP2Y  Vo → 10kΩ + 10kΩ → GPIO32    Vout@5V = 2.5V ✓
//
// ─── BLYNK VIRTUAL PINS ──────────────────────────────────────────────────────
//   FAST (5s):  V2  CO ppm      V6  CO phase    V7  CO raw
//               V12 GPS lat     V13 GPS lng      V14 GPS sats
//               V15 Humidity    V16 Temperature  V17 HDOP
//               V19 Eng msg     V20 Status flags V21 RSSI dBm
//               V22 WiFi qual%
//   SLOW (10s): V1  Sound dB    V3  UVI          V4  Diagnostic
//               V5  TVOC ppb    V8  Roll         V9  Pitch
//               V10 Yaw         V11 IMU temp     V18 Dust mg/m³
//               V23 eCO2 ppm    V24 AQI          V25 Heater V (A1 feedback)
//               V34 ESP reset reason (diagnostic)
//
//               Diagnostic:
//               V35 — cumulative PPS event counter
//               V36 — milliseconds since last PPS
//               V37 — maximum Arduino loop() gap
//               V38 — maximum sensor-task cycle time
//               V39 — maximum Blynk.run() duration
//               V40 — maximum GPS processing duration
//               V41 — maximum measured I²C sensor-path duration
//               V42 — maximum FAST telemetry duration
//               V43 — maximum SLOW telemetry duration
//
//               V45 — last I²C device  (0 = none/unknown, 1 = ENS160, 2 = AHT21, 3 = BMI160)
//               V46 — last I²C operation (0 = none/unknown, 1 = read, 2 = measurement/command, 3 = configuration)
//               V47 — cumulative I²C transaction error count
//               V48 — maximum AHT21 read duration, ms
//               V49 — maximum ENS160 operation/read duration, ms
//               V50 — maximum BMI160 raw-read duration, ms
//               V52 — last Core-1 execution stage
//               V53 — maximum Core-1 stage duration, ms
//               V54 — Core-1 stall event count
//               V56 — last Blynk TX V-pin
//               V57 — last Blynk virtualWrite() duration, ms
//               V58 — maximum Blynk virtualWrite() duration, ms
//               V59 — cumulative Blynk virtualWrite() call count
//               V60 — last Blynk TX value length
//               V63 — queued TX item count
//               V64 — last transmitted V-pin
//               V65 — maximum virtualWrite() duration
//
// ─── ARCHITECTURE ────────────────────────────────────────────────────────────
//   Core 0 (pro_cpu) — Arduino loop/events: WiFi, lwIP, Blynk.run(), BlynkTimer
//   Core 1 (app_cpu) — sensorTask: all sensors, drivers and CO state machine
//   Core 1 publishes complete SensorData snapshots through a one-slot queue;
//   Core 0 only peeks that queue and never invokes a sensor driver.
//   Blynk.connect(timeout) NOT used — avoids IWDT crash on Core 1.
//
// ─── v2.1 CHANGES ────────────────────────────────────────────────────────────
//   • Replaced Adafruit_ENS160 with ScioSense_ENS160 library (correct library
//     for this sensor module). API calls updated throughout.
//   • Fixed "ADC: CONFLICT driver_ng" error:
//     Root cause: analogSetAttenuation() in Arduino core 3.x internally calls
//     the new adc_oneshot (driver_ng) API. The legacy I2S driver installs its
//     own ADC handle via the legacy ADC API. Both cannot coexist — firmware
//     aborts with "CONFLICT! driver_ng is not allowed to be used with the
//     legacy driver".
//     Fix: replaced analogSetAttenuation() with analogSetPinAttenuation()
//     called individually for each ADC pin. Per-pin calls do NOT invoke
//     driver_ng and are safe alongside the legacy I2S driver.
//     I2S (initINMP441) is also installed LAST in setup(), after all ADC
//     pin configuration is complete.
//
// ─── v2.2 CHANGES — DEFINITIVE FIX FOR ADC/I2S COLLISION ───────────────────
//   The v2.1 fix above (per-pin attenuation + install-order) reduced the
//   frequency of the "ADC: CONFLICT driver_ng is not allowed to be used
//   with the legacy driver" abort but did not eliminate it, because the
//   TRUE root cause is not ordering — it is that <driver/i2s.h> (legacy
//   I2S driver) and analogRead()/analogSetPinAttenuation() (which use the
//   new driver_ng ADC API on ESP32 Arduino core 3.x) belong to two
//   different, mutually-exclusive ESP-IDF driver families. Once the
//   legacy I2S driver is installed, ANY subsequent driver_ng ADC call
//   (including calls made later inside sensorTask, e.g. every CO/UV/dust
//   analogRead()) can abort — no init ordering fixes this permanently.
//
//   HARMONIZED FIX: migrated the INMP441 microphone driver from the legacy
//   <driver/i2s.h> API (i2s_driver_install / i2s_set_pin / i2s_read) to the
//   new standard driver <driver/i2s_std.h> API (i2s_new_channel /
//   i2s_channel_init_std_mode / i2s_channel_enable / i2s_channel_read).
//   The new i2s_std driver is part of the same "new driver" family as
//   driver_ng ADC and never touches the ADC peripheral or its driver
//   state at all — I2S and ADC are fully independent hardware blocks in
//   this API family, so no conflict is possible in either init order or
//   at runtime. This is the same class of fix as replacing legacy ADC
//   calls with driver_ng equivalents: harmonizing both libraries onto the
//   same modern driver generation removes the collision at its source
//   instead of merely avoiding it by sequencing.
//
//   All CO/UV/dust ADC logic, EMA smoothing, closed-loop heater control,
//   GPS/IMU/ENS160/AHT2x logic, Blynk architecture, and pin map are
//   UNCHANGED from v2.1 — only the internal implementation of
//   initINMP441() and readINMP441_dB() changed to use the new driver.
//   The public behaviour (function signatures, return values, dB output
//   range, SENSOR_UNAVAILABLE sentinel) is identical.
//
// ─── v2.3 CHANGES — GPS PPS + GPS/IMU INERTIAL NAVIGATION (DEAD RECKONING) ──
//   1) GPS 1PPS (pulse-per-second) output wired to GPIO5 (free since DHT21
//      was removed in v2.0) via a rising-edge interrupt. PPS gives a hard
//      diagnostic of whether the GPS has a real timing lock (a GPS can
//      output stale/repeated NMEA without a genuine fix; a clean ~1.000s
//      PPS train is a much stronger lock indicator). Reported on Blynk V26.
//
//   2) New GY-BMI160 6-DOF IMU (I2C, shares the ENS160/AHT2x bus on
//      GPIO21/22, address 0x69) driven with a minimal register-level driver
//      (no external library dependency — BMI160 registers are simple and
//      well documented, and a hand-rolled driver avoids adding another
//      library that could collide with the ADC/I2S drivers the way v2.1/2.2
//      had to work around). Provides raw gyro (dps) and accel (g) samples.
//
//   3) New function inertialNav() fuses GPS position (and, when available,
//      GPS-derived speed+course from RMC/VTG) with BMI160 gyro/accel using
//      a compact 4-state Extended Kalman Filter (state = East, North,
//      Speed, Heading — a standard "2D ground-vehicle EKF" pattern widely
//      used for exactly this GPS+MEMS-gyro dead-reckoning problem on
//      resource-constrained MCUs). Design rationale, taken from common
//      embedded GNSS/INS fusion practice:
//        • Position is NOT double-integrated from the accelerometer.
//          Double-integrating cheap MEMS accelerometer noise/bias is a
//          well-known way to get position error that grows with t^2 —
//          essentially unusable within seconds. Instead, velocity is
//          treated as a slowly-varying state that is corrected directly
//          by GPS (position deltas AND, when moving fast enough, GPS
//          Doppler speed) and held constant between fixes.
//        • Heading is propagated by integrating BMI160 gyro Z between GPS
//          fixes (classic gyro dead-reckoning) and corrected by GPS course
//          only when GPS speed exceeds INAV_MIN_COURSE_SPEED_MPS, because
//          GPS course-over-ground is meaningless / noisy near zero speed.
//        • Gyro bias is NOT estimated inside the main filter (that would
//          need it to be observable, which a position/speed/heading-only
//          GPS update does not cleanly provide). Instead it is calibrated
//          separately using a Zero-velocity Update (ZUPT): whenever the
//          accelerometer magnitude reads ~1g and the gyro reads ~0 dps for
//          a sustained period, the platform is known to be stationary, so
//          the live gyro Z reading during that window IS the bias — a
//          textbook robust technique for MEMS gyro bias tracking that is
//          far simpler and more reliable than trying to make the bias
//          observable inside the position/velocity/heading EKF.
//        • ZUPT also forces the velocity state back toward zero via a
//          tiny-variance pseudo-measurement, preventing the classic
//          "GPS jitter looks like slow drift" creep while parked.
//      All matrix operations are hand-written fixed-size (4x4 max) loops —
//      no dynamic allocation, no external linear-algebra library — so the
//      filter is cheap enough to run at the user-configurable output rate
//      (inav_update_hz, default 10 Hz) on Core 0 alongside every other
//      sensor read without measurably affecting loop timing.
//
//   4) GPS-loss state machine (independent of, and reported alongside, the
//      existing STATUS_GPS_FAULT bit which is unchanged):
//        NO_FIX_YET  — no GPS fix has ever been received since boot;
//                      inertialNav() is idle until an origin can be set.
//        GPS_FIX     — a fresh GPS fix was just used to correct the filter.
//        IMU_RECENT  — GPS fix missing, but fewer than inav_stale_after_
//                      misses "expected" fixes (based on the adaptively
//                      measured GPS update interval) have been missed.
//                      Interpolated position is trusted almost as much as
//                      a real fix.
//        IMU_STALE   — GPS has been missing longer than that, but still
//                      within inav_max_loss_minutes. Interpolation
//                      continues but accuracy is explicitly flagged as
//                      degrading.
//        LOST        — GPS missing beyond inav_max_loss_minutes.
//                      Interpolation is HALTED (position held, not
//                      advanced further) until GPS reacquires, exactly as
//                      requested, to avoid reporting arbitrarily-drifted
//                      dead-reckoned position after a long outage.
//      Engineering messages are emitted only on state TRANSITIONS (never
//      every loop) so they don't drown out other diagnostics.
//
//   5) New Blynk virtual pins (all additive — no existing pin renumbered):
//      V26 PPS locked (0/1)         V27 INAV state (0-4, see enum above)
//      V28 INAV latitude            V29 INAV longitude
//      V30 INAV altitude (m, held from last valid GPS fix — BMI160 has no
//          barometer, so altitude is intentionally NOT dead-reckoned)
//      V31 INAV speed (m/s)         V32 INAV heading (deg, 0=N, clockwise)
//      These are entirely separate from the existing raw-GPS V12/V13 —
//      V12/V13 still report exactly what they did before (raw GPS lat/lng,
//      snapped to 0 or offset when no fix, as originally implemented).
//
// ─── v2.4 CHANGES — BMI160 REPLACES JY-901 ENTIRELY [SUPERSEDES v2.3] ───────
//   JY-901/WT901 (UART2, GPIO16/17) has been removed completely. All earlier
//   references to it in this file (the v2.0 "KEPT" list, the v2.3 changelog
//   text saying "existing JY-901 ... is untouched", the old PIN MAP entries
//   for GPIO16/17) are now historical — BMI160 is the only IMU in the system.
//   GPIO16/GPIO17 are free again (no longer used for anything).
//
//   BMI160 now does double duty:
//     1) Still the sole inertial source for inertialNav() (unchanged EKF).
//     2) ALSO now feeds V8 Roll / V9 Pitch / V10 relative IMU yaw / V11 IMU temperature —
//        the job JY-901's onboard fusion used to do — via a lightweight
//        complementary filter computed right here, since raw BMI160 has no
//        onboard sensor fusion of its own:
//          • Roll/Pitch: classic 98/2 complementary filter — gyro-integrated
//            angle corrected toward the gravity-vector accelerometer angle
//            each cycle. Gravity gives an absolute, non-drifting reference
//            for these two axes, which is why a simple complementary filter
//            (rather than a full AHRS) is sufficient and cheap here.
//          • Yaw: BMI160 has no magnetometer, so V10 is now a RELATIVE gyro-Z
//            yaw that is useful for detecting rotation but will drift over long
//            periods. Absolute navigation heading remains in INAV/EKF.
//            V61 uses the same relative IMU yaw value as V10.
//            being maintained and GPS-corrected inside inertialNav() — one
//            state, two consumers, no drift, no extra code.
//          • IMU temperature: read from BMI160's internal die-temperature
//            registers (0x20/0x21) — informational only, expect it to read
//            a few degrees above ambient due to self-heating.
//     STATUS_IMU_OK / STATUS_IMU_STUCK now reflect BMI160 I2C read health
//     (a failed/absent BMI160 read) instead of JY-901 UART frame timeout —
//     same two status bits, same meaning to anything consuming V20.
// ============================================================================
// v2.7 LONG-TERM RELIABILITY IMPROVEMENTS — additive, existing behaviour kept
//   • GPS PPS ISR now records only timing/event state; D2 LED control runs in
//     sensorTask so the LED reflects the normal application context as well as
//     PPS activity. Atomic event exchange prevents PPS events being lost.
//   • WiFi reconnect was changed to a non-blocking 20s association state
//     machine; the original blocking wifiConnect() helper remains available.
//   • sensorTask heartbeat + ESP task-watchdog subscription provide an actual
//     recovery path for a stalled sensor task. The existing optional 10-minute
//     auto-reset remains intact and disabled by default.
//   • Periodic heap/minimum-heap/largest-free-block health telemetry is emitted
//     once per minute for long-duration burn-in and memory-fragmentation checks.
//   • Existing sensor, GPS, INAV, CO, Blynk, map, display, buzzer, and status
//     logic is preserved; changes are limited to reliability/monitoring paths.
// ============================================================================
// v3.9.90 COHERENT SENSOR SNAPSHOT + PRIORITY V4
//   - Keeps the existing stock BlynkSimpleEsp32 transport and the v3.9.89
//     safe offline/connected architecture.
//   - V4 remains scheduled at exactly the existing 1000ms BlynkTimer cadence.
//     Its queued value is given priority by blynkTxService(), so V4 gets the
//     next available TX slot instead of sitting behind ordinary telemetry.
//   - At the same 1s timer tick, one SensorData snapshot is captured and
//     serialized to V61 as one compact CSV record containing the sensor set,
//     GPS timestamp, device millis timestamp, GPS position, INAV position and
//     status/network context. This gives the database/GIS backend one coherent
//     record rather than requiring joins based on network arrival times.
//   - Existing V1-V60 datastreams are preserved for the current Blynk dashboard.
//     V61 is additive and is the preferred channel for coherent database/GIS
//     ingestion.
//   - V61 is intentionally a string datastream: Blynk 0.6.1 can transmit it as
//     one virtualWrite, but Blynk does not automatically split a multi-field
//     application record into separate numeric datastreams. The backend should
//     parse the documented CSV field order.
//   - No direct V4 virtualWrite remains in blynkSendV4(); all application writes
//     still pass through blynkTxService().
//
// v3.9.19 BLYNK TX CALL DIAGNOSTICS + WIFI SLEEP SWITCH — additive
//   • Added START/END serial instrumentation around the actual Blynk.virtualWrite() call.
//   • Records V-pin, value length, queue depth, slot, call duration and maximum duration.
//   • Added dedicated Core-1 stages for normal and map virtualWrite() calls.
//   • Added cumulative TX diagnostics to the periodic health output.
//   • Restored WiFi modem-sleep as the default, controlled by WIFI_SLEEP.
//     Set WIFI_SLEEP=false to disable modem-sleep for comparison testing.
//   • No telemetry intervals, queue limits, watchdog, heartbeat or recovery logic changed.
// ============================================================================
// v3.9.21 RAW NON-BLOCKING BLYNK TRANSPORT — additive
//   • Replaced the Blynk transport's underlying WiFiClient write/read path with
//     a direct lwIP TCP socket owned by the Blynk transport.
//   • Socket is permanently O_NONBLOCK; connect/write/read are gated by
//     select() with hard millisecond budgets.
//   • Write budget is 250ms, connect budget 300ms, read budget 1s.
//   • A failed/timeout write closes the socket immediately, preventing the
//     ESP32 Arduino NetworkClient 10 x 1s write-retry loop that caused the
//     observed ~10.01s Core-1 stall.
//   • Existing Blynk protocol, TX queue, heartbeat, WiFi and recovery logic
//     remain unchanged.
// ============================================================================
// ============================================================================
// v3.9.26 DISABLE DUPLICATE WiFi AUTO-RECONNECT — ROOT-CAUSE FIX FOR
// REPEATED ASSOC_LEAVE / HANDSHAKE-TIMEOUT CASCADE
//   • ROOT CAUSE FOUND: field logs showed WiFi HEALTH eventCount growing
//     far faster than actual reconnect attempts (e.g. eventCount=13 for
//     only attempts=3), with wifiLastDisconnectReason=8
//     (WIFI_REASON_ASSOC_LEAVE — the station voluntarily leaving its own
//     association, not a router-side kick). This lined up exactly with
//     repeated "[Blynk] Handshake timeout" events (three in a row, ~25s
//     apart, matching BLYNK_HANDSHAKE_TIMEOUT_MS + BLYNK_CONNECT_RETRY_MS)
//     that ultimately cascaded into the existing WiFi-reset-then-
//     esp_restart() escalation path — a full device reboot triggered by a
//     radio issue, not a Blynk server or app issue.
//   • WiFi.setAutoReconnect(true) was enabling the ESP32 WiFi driver's OWN
//     independent reconnect logic on every disconnect, running
//     concurrently with and unaware of this firmware's custom
//     wifiService()/wifiStartConnect() state machine. Two reconnect
//     mechanisms racing on the same radio is a known cause of exactly this
//     churn: one path can reassociate mid-way through what the other just
//     built, invalidating any TCP session (including an in-progress Blynk
//     handshake) that happened to be establishing at that moment — which
//     looks identical to a slow/hung server from the application's point
//     of view, even though the actual network and Blynk server were both
//     healthy throughout (confirmed via a second ESP8266 Blynk device on
//     the same network that never exhibits this failure).
//   • FIX: WiFi.setAutoReconnect(false). wifiService() already owns all
//     reconnect timing/backoff explicitly and is now the only mechanism
//     that decides when to reassociate.
// ============================================================================
// v3.9.24 NON-BLOCKING TCP PROBE — ROOT-CAUSE FIX FOR REPEATED BLYNK DROPS
//   • ROOT CAUSE FOUND: blynkServerReachable() (the pre-handshake TCP
//     reachability probe) called lwip_select() with a timeout of up to
//     several seconds, entirely BLOCKING Core 1 — the same core that must
//     keep calling Blynk.run() to service the live protocol session and
//     read inbound heartbeat/PING responses. Every probe attempt (fired
//     whenever Blynk was not currently connected) froze Core 1 for the
//     full probe duration, during which any inbound server response went
//     unread. This directly explains the "Heartbeat timeout"/repeated
//     "hardware joined" session-churn pattern seen in production even with
//     stable RSSI and a healthy LAN/router (confirmed via a second, always-
//     reliable ESP8266 Blynk board on the same network that never exhibits
//     this failure — ruling out router/WiFi RF causes).
//   • FIX: replaced the single blocking blynkServerReachable() call with a
//     non-blocking state machine — blynkProbeStart() kicks off a
//     non-blocking connect() and returns immediately; blynkProbePoll() is
//     called once per loop() pass and uses select() with a ZERO timeout, so
//     it costs microseconds regardless of connection state. The probe now
//     spans multiple loop() passes while Blynk.run(), sensor mutex
//     service, and everything else in loop() continues to execute normally
//     in between polls — Core 1 is never stalled waiting on the probe.
//   • Core-1 stage tracing (CORE1_STAGE_TCP_PROBE) now brackets only the
//     brief per-pass poll rather than the whole multi-pass wait, so the
//     stall monitor and V52/V53 diagnostics correctly reflect actual time
//     spent, not wall-clock probe duration.
//   • BLYNK_SERVER_PROBE_TIMEOUT_MS and BLYNK_PROBE_FAILS_BEFORE_WIFI_RESET
//     were also loosened (6000ms / 6 fails, from 3000ms / 3 fails) as a
//     secondary safety margin, though the non-blocking rewrite above is the
//     actual fix for the observed failure mode.
//   • SECOND ROOT CAUSE FOUND (same version, added after further field
//     testing): WRITE_BUDGET_MS was 250ms — far too tight for real WiFi
//     jitter. A single write timing out by 1ms (errno=EAGAIN, a transient,
//     non-fatal condition) was tearing down an otherwise healthy
//     connection. Raised to 1200ms; still far below any watchdog concern
//     since waitForSocket() blocks on select(), which yields to the
//     scheduler rather than spinning.
//   • THIRD ROOT CAUSE FOUND (same version, added after further field
//     testing): BoundedBlynkArduinoClient::connected() previously closed
//     the socket and reported "disconnected" SILENTLY (no log line at all)
//     on ANY errno from a MSG_PEEK probe other than EAGAIN/EWOULDBLOCK/
//     EINTR — this is the most likely explanation for "Connection lost"
//     events with no preceding WRITE/READ TIMEOUT or "peer closed" message
//     anywhere in the log. Now only genuinely fatal errno values
//     (ECONNRESET, ENOTCONN, EPIPE, EBADF) close the socket; every other
//     errno is logged and treated as transient, leaving the actual
//     dead-connection detection to read()/write(), which already have
//     their own bounded, logged timeout paths.
// ============================================================================
// v3.9.23 NETWORK + I2C RECOVERY DIAGNOSTICS — additive
//   • Restored the v3.7 I2C failure-isolation layer that was lost during the
//     v3.9.x network-diagnostics merge: device preflight, bounded bus recovery,
//     and BMI160 transaction isolation.
//   • Added I2C SDA/SCL line-state diagnostics and correlation with the CO heater
//     phase/duty so electrical/power/bus interactions can be identified.
//   • Confirmed and preserved the historical pin move: GPS RX moved from GPIO22
//     to GPIO13 so GPIO22 remains dedicated to I2C SCL.
//   • Added WiFi event disconnect-reason capture, AP/channel/BSSID diagnostics,
//     consecutive association-failure tracking and a bounded WiFi recovery path.
//   • Added a last-resort software restart after persistent WiFi outage instead
//     of allowing the station to remain offline indefinitely.
//   • Corrected raw TCP diagnostic errno capture so the original write/select
//     failure errno is not overwritten by later getsockopt()/getsockname() calls.
//   • WiFi modem sleep is OFF in this diagnostic build to remove radio power-save
//     latency/current variation as a confounding variable.
// ============================================================================
// v3.9.22 DEEP NETWORK DIAGNOSTICS — additive
//   • Added one user switch (DEBUGON) controlling the verbose diagnostic output.
//   • Added detailed raw TCP socket diagnostics: SO_ERROR, SO_SNDBUF, local/remote
//     ports, socket age, WiFi RSSI/IP/gateway and write failure errno.
//   • Added TCP keepalive configuration where supported by the ESP32/lwIP build.
//   • Added counters for EAGAIN, timeouts, socket errors and TCP reconnects.
//   • Added a compact 10-second diagnostic summary so a failure can be diagnosed
//     from one serial capture instead of adding another temporary instrumentation
//     version.
//   • All high-volume diagnostic lines are suppressed when DEBUGON=false.
//   • Existing application, sensor, INAV, Blynk queue and recovery behaviour is kept.
// ============================================================================
// ============================================================================
// v3.9.18 CORE 1 STALL DIAGNOSTICS — additive
//   • Added Core-1 stage tracing around WiFi, TCP probe, Blynk.run(), heartbeat,
//     BlynkTimer, TX queue and health/checkpoint processing.
//   • Added an independent Core-1 monitor task. If Arduino loop() stops making
//     progress for >500ms, it reports the exact last stage, elapsed time, WiFi/Blynk
//     state, sensor-task heartbeat and heap. This is diagnostic only and does not
//     reset the ESP32.
//   • Added maximum Core-1 stage duration and stall count diagnostics (V52-V54).
//   • Existing Blynk heartbeat, TX queue, WiFi sleep and recovery logic preserved.
// ============================================================================
// v3.9.17 BLYNK HEARTBEAT / WIFI RELIABILITY FIX — additive
//   • Added an explicit Blynk protocol PING every 20 seconds while connected.
//     This is independent of telemetry and is exempt from BLYNK_MSG_LIMIT.
//   • Added the missing WiFi.setSleep(false) call. Earlier comments claimed
//     modem-sleep was disabled, but the actual call was absent from the file.
//     Keeping WiFi power-save disabled removes a known source of intermittent
//     LAN packet latency/loss for long-lived TCP sessions.
//   • Re-asserted WiFi.setSleep(false) after association and enabled the ESP32
//     WiFi auto-reconnect flag.
//   • Added serial heartbeat counters so a future failure can distinguish
//     "no PING sent" from "PING sent but no Blynk response arrived".
//   • Existing bounded Blynk transport, TX queue, WDT and recovery logic are
//     preserved.
// ============================================================================
// v3.9 REMOTE HARD RESET — additive, existing behaviour kept
//   • Blynk Virtual Pin V55 is a button that requests the strongest
//     firmware-triggered ESP32 reset available through the application.
//   • The reset occurs only when Blynk delivers HIGH/1 to V55.
// v3.9.1 note: the earlier V55 reset was deferred until loop() was outside
// Blynk.run(); that implementation also never disabled interrupts and added
// explicit Blynk/WiFi teardown. v3.9.2 simplifies the reset to the direct
// esp_restart() path below, with interrupts enabled, as required by the ESP32
// restart machinery.
//   • The reset path invokes esp_restart() so the ESP32 performs a full system
//     software restart through ESP-IDF. Interrupts are NOT disabled by the sketch.
//   • This is a firmware reset, not a physical power-cycle/brownout reset.
// ============================================================================
// // v3.8 RTC CRASH DIAGNOSTICS — additive, existing behaviour kept
//   • V51 now publishes the previous-run RTC checkpoint as a latched diagnostic
//     string. The snapshot is captured before the new run starts updating RTC.
//   • V51 is never populated from the live/current-run checkpoint, so it does not
//     drift while a person is reading or recording the crash information.
//   • The same latched V51 string may be re-published by the existing 10-second
//     diagnostic pathway; its contents remain identical until the next reboot.
//   • RTC checkpoint storage remains in ESP32 RTC SRAM only; no flash writes are
//     introduced by this diagnostic feature.
// ============================================================================
// v3.7 LONG-TERM RELIABILITY IMPROVEMENTS — additive, existing behaviour kept
//   • Added ESP32 reset-cause diagnostics at startup, including human-readable 
//     reset reason reporting.
//   • Added Blynk V34 diagnostic output for the ESP32 reset reason.
//   • Added explicit ESP32 task-watchdog configuration with a 10-second timeout 
//     instead of relying on the Arduino core default.
//   • Added explicit watchdog registration and feeding for the sensor task only 
//     after a complete sensor cycle.
//   • Added sensor-task heartbeat monitoring to detect a task that is alive but 
//     no longer completing its work cycle.
//   • Added asynchronous non-blocking buzzer control so buzzer tones no longer 
//     block sensor processing or Blynk/WiFi operation.
//   • Added an asynchronous buzzer pattern state machine for multi-beep alerts 
//     without using blocking delays.
//   • Added persistent sensor-health monitoring for the INMP441 microphone with 
//     fault and recovery reporting.
//   • Added persistent sensor-health monitoring for the ML8511 UV sensor with fault 
//     and recovery reporting.
//   • Added persistent sensor-health monitoring for the GP2Y1010 dust sensor with 
//     fault and recovery reporting.
//   • Added `SENSOR_UNAVAILABLE` handling for failed INMP441, UV, and dust sensors 
//     so invalid sensors do not provide misleading measurements.
//   • Added fault persistence thresholds to prevent a single bad sensor reading 
//     from generating a false alarm.
//   • Added automatic sensor-recovery reporting when a previously failed sensor 
//     starts providing valid data again.
//   • Ensured sensor-health faults are isolated so a malfunctioning sensor does not 
//     stop the sensor task or prevent the remaining sensors from operating.
//   • Preserved all existing sensor-specific fault detection, retry, watchdog, 
//     GPS, CO, Blynk, WiFi, display, LED, buzzer, and status logic.
//   • Periodic [PERF] Serial report
//   • v3.9.3: Serialized all sketch-level LEDC operations across Core 0 and Core 1
//     to prevent a cross-core LEDC driver race that can trigger the CPU1 Interrupt-WDT.
//
// ============================================================================


// v3.9.131: raised from 20 -> 120 seconds. Field logs across this whole
// investigation consistently show: RSSI excellent throughout (-46 to
// -58dBm), zero WiFi association drops (wifiDiscEvents=0 in every log),
// and V61 TX (actual sensor data) never failing — no writeTimeout,
// writeEAGAIN, or socketErr, even for 600+ seconds immediately before a
// disconnect. The heartbeat-timeout deaths we chased were this deadline
// (BLYNK_HEARTBEAT*1000 + 3*BLYNK_TIMEOUT_MS, previously 23000ms) firing
// during ordinary quiet periods on a connection that was never actually
// dead — not a symptom of a broken link. This constant does not affect
// data delivery: virtualWrite (V61 TX) and the server's push to the
// Blynk app are independent of it. Raising it only makes the client more
// tolerant of quiet stretches before deciding to reconnect; it does not
// disable dead-connection detection, so a genuinely dead socket is still
// caught, just after a longer wait (now ~123s instead of ~23s).
//#define BLYNK_HEARTBEAT 120
// v3.9.141: heartbeat 10s -> 30s. Library heartbeat deadline is
// BLYNK_HEARTBEAT*1000 + 3*BLYNK_TIMEOUT_MS; with 10s/500ms that is 11.5s, but
// Core 1 is regularly blocked ~10.0s (BlynkRunMax/Core1Max=10012ms), so ANY
// stall tripped "Heartbeat timeout" and tore the session down.
// v3.9.176: the 47-58s retransmission captured on this ESP32 disproves the
// 19s deadline. Keep frequent path probes separate from the long deadline.
#define BLYNK_HEARTBEAT 180
#define BLYNK_PROTOCOL_PING_INTERVAL 10
#define BLYNK_PROTOCOL_PING_RETRY_MS 10000UL
#define BLYNK_APPLICATION_ACK_PARTIAL_FLUSH_MS 0UL

// v3.9.57 diagnostic-only reset/server health counters. These never trigger recovery.
// Forward declarations: full definitions live later in the file (WiFi event
// handler section, ~line 7232/7249), but blynkStockRxForensicSessionEnd()/
// blynkStockRxForensicService() (added in v3.9.126 for RSSI/disconnect-event
// logging) are defined earlier and need these visible at that point.
extern volatile uint8_t wifiLastDisconnectReason;
extern volatile uint32_t wifiDisconnectEventSequence;
volatile uint32_t diagResetRequests = 0;
volatile uint32_t diagServerProbeStarts = 0;
volatile uint32_t diagServerProbeSuccess = 0;
volatile uint32_t diagServerProbeFails = 0;
volatile uint32_t diagServerProbeMaxMs = 0;
volatile uint32_t diagBlynkHandshakeStarts = 0;
volatile uint32_t diagBlynkHandshakeMaxMs = 0;
volatile uint32_t diagBlynkRunCalls = 0;
volatile uint32_t diagBlynkRunSlow = 0;
volatile uint32_t diagBlynkRunMaxMs = 0;
volatile uint32_t diagBlynkTxSuccess = 0;
volatile uint32_t diagBlynkTxFail = 0;
volatile uint32_t diagBlynkTxTimeout = 0;
volatile uint32_t diagBlynkTxMaxMs = 0;
volatile uint32_t diagBlynkRxEvents = 0;
volatile uint32_t diagBlynkLastRxAgeMs = 0;
volatile uint32_t diagBlynkSocketAgeMs = 0;
volatile int32_t diagBlynkLastErrno = 0;
volatile uint32_t diagBlynkMaxQueue = 0;

static char diagLastResetReason[48] = "none";

// v3.9.19: fine-grained diagnostics for the actual Blynk.virtualWrite() call.
// These declarations intentionally appear before perfReport(), because the
// Arduino prototype generation can otherwise make perfReport() see them only
// after its definition.
volatile uint32_t blynkTxCallCount = 0;
volatile uint32_t blynkTxLastCallMs = 0;
volatile uint32_t blynkTxMaxCallMs = 0;
volatile int16_t  blynkTxLastVPin = -1;
volatile uint16_t blynkTxLastValueLen = 0;

static void diagnosticResetRequest(const char *reason) {
  diagResetRequests++;
  snprintf(diagLastResetReason, sizeof(diagLastResetReason), "%s", reason ? reason : "unknown");
#if BLYNK_FORENSIC_LOCK
  if (xPortGetCoreID() == 0)
    Serial.printf("[RESET BLOCKED] would request esp_restart: %s | count=%lu | millis=%lu\n",
                  diagLastResetReason, (unsigned long)diagResetRequests,
                  (unsigned long)millis());
#else
  if (xPortGetCoreID() == 0)
    Serial.printf("[RESET] esp_restart requested: %s | count=%lu | millis=%lu\n",
                  diagLastResetReason, (unsigned long)diagResetRequests,
                  (unsigned long)millis());
  delay(20);
  esp_restart();
#endif
}

// v3.9.89: STOCK WiFiClient library-side read budget was retained at 1s.
// This is diagnostic/library configuration only; it is NOT used as a promise
// that Blynk.run() or Blynk.virtualWrite() cannot block longer internally.
// v3.9.140: Reduced from 1000ms to 500ms to be more aggressive about processing
// inbound server responses during sensor-heavy loop iterations.
// v3.9.141: back to 2000ms (library default). 500ms is ALSO the Blynk login
// timeout (login response must arrive within this after HW_LOGIN is sent) and
// part of the heartbeat deadline; it does not make run() process data faster.
// v3.9.170: 3000ms matches the stable ESP8266/Blynk 0.6.1 default.
#define BLYNK_TIMEOUT_MS 3000UL

// v3.9.148: Arduino-ESP32 3.3.12 stores NetworkClient::_timeout in
// milliseconds. BlynkArduinoClient already applies BLYNK_TIMEOUT_MS when it
// receives the client, and the connection-edge hook below reasserts it.
// This timeout does NOT bound NetworkClient::write(): core 3.3.12 has a fixed
// 100 x 100ms EAGAIN retry loop. BoundedWriteWiFiClient below replaces only
// that write method with an explicit deadline.
#define BLYNK_STOCK_WRITE_BUDGET_MS 750UL  // legacy diagnostic threshold only; writes are non-blocking in v3.9.162
#define BLYNK_STOCK_CONNECT_BUDGET_MS 1000UL
#define BLYNK_STOCK_DEFERRED_TX_BYTES 2048U
// v3.9.185: the v3.9.184 field run proved that a fresh socket authenticates in
// about one second while preserving the old sequence space causes a 65s outage.
// Normal RESPONSE gaps remain below about 20s, but packet captures also proved
// that an accepted ESP32 TCP segment can need 47-58s of retransmission before
// its RESPONSE returns.  Allow 70s with bulk HTTP quiesced before discarding
// the affected Blynk socket; lower reset domains remain separately gated.
#define BLYNK_SERVER_STALL_GRACE_MS           70000UL
#define BLYNK_STOCK_PATH_RECOVERY_ESCALATE_MS BLYNK_SERVER_STALL_GRACE_MS
#define BLYNK_APPLICATION_PROBE_TIMEOUT_MS    70000UL
// HTTP is isolated after either exact application or protocol proof has been
// unanswered for 3s. That early isolation provides the quiet interval needed
// for the longer evidence-based TCP retransmission allowance above.
#define BLYNK_PROTOCOL_PROBE_TIMEOUT_MS       15000UL
#define BLYNK_STOCK_PATH_RECOVERY_VERIFY_MS   30000UL
#define BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS 3U
#define BLYNK_SEQUENCE_HOLE_SERVER_COOLDOWN_MS 30000UL
#define BLYNK_SHARED_PATH_CORROBORATION_MS      60000UL
#define BLYNK_APPLICATION_CANARY_CHECKPOINTS   8U

// Hooks consumed by the locally patched BlynkProtocol.h. Forward declarations
// are sufficient here; definitions follow the concrete transport instance.
static void blynkProtocolTransportService();
static bool blynkProtocolTransportPending();
static void blynkPurgeClosedSessionPcbs(const char *source);
#define BLYNK_TRANSPORT_AWARE 1
#define BLYNK_TRANSPORT_SERVICE() blynkProtocolTransportService()
#define BLYNK_TRANSPORT_TX_PENDING() blynkProtocolTransportPending()
#define BLYNK_TRANSPORT_PENDING_TIMEOUT_MS BLYNK_STOCK_PATH_RECOVERY_ESCALATE_MS
#define BLYNK_APPLICATION_ACK_BARRIER 1

// v3.9.117: disable Blynk v0.6.1 internal application-message limiter.
// Its sendCmd() limiter calls Blynk.run() recursively while waiting for the
// next slot. Our coalescing TX scheduler already spaces application writes,
// so the library must not introduce a second nested run()/wait loop.
#define BLYNK_MSG_LIMIT 0

// v3.9.54: diagnostic/recovery guard for Core 1.  The stock transport is tuned to a 1s socket
// timeout.  This guard does NOT shorten the socket
// timeout; it records when a complete Blynk.run() takes unusually long and
// forces a clean session teardown after that call returns, preventing repeated
// long Blynk.run() passes from chaining together.
#define BLYNK_RUN_GUARD_MS 900UL

#include "secrets.h"
// secrets.h: #define WIFI_SSID / WIFI_PASS / BLYNK_AUTH / BLYNK_SERVER / BLYNK_PORT

// ============================================================================
// v3.9.87: STOCK BLYNK ESP32 TRANSPORT
//
// This firmware deliberately uses the transport supplied by Blynk v0.6.1:
//
//   BlynkProtocol
//       -> BlynkArduinoClient
//       -> WiFiClient
//       -> ESP32 WiFi/lwIP stack
//
// Our application still owns WiFi association/reconnection and continues to
// use Blynk.config(). Blynk.begin() is NOT used because it would make the
// Blynk library own the WiFi connection and can block indefinitely.
//
// The previous raw-lwIP BoundedBlynkArduinoClient implementation is retained
// below only as historical A/B-test code, but CONNECTION_MODE is fixed to 1 so
// it is NOT the active Blynk transport in this firmware.
// ============================================================================
#ifndef CONNECTION_MODE
  // v3.9.130: REVERTED. v3.9.129 flipped this to 0 based on a misreading —
  // the BoundedBlynkArduinoClient class is well-formed and heavily
  // instrumented, which looked like abandoned-in-progress work, but line
  // ~1469 of this file carries an explicit #error guard from v3.9.88
  // stating the custom raw-lwIP transport was deliberately retired and the
  // stock BlynkSimpleEsp32 transport is the intended, supported path. The
  // custom class remains in the source for reference only. Restored to 1.
  // Every "stock-retry" stall diagnosed this session (2.7-3.0s Blynk.run()
  // blocks, later resolved by the Serial TX buffer fix) was correctly
  // diagnosed against the real, active transport — that conclusion stands.
  // What's still open is the heartbeat-timeout churn (server goes quiet for
  // 20+s), which needs to be investigated against the STOCK WiFiClient
  // path, not BoundedBlynkArduinoClient.
  #define CONNECTION_MODE 1
#endif

// v3.9.43: field diagnostics of the WiFi/Blynk stall issue are complete —
// root cause confirmed as server-side (JVM GC/allocation pressure), fixed
// via blynk.service G1GC tuning. Set back to false for normal operation:
// this silences [Blynk RAW]/[Blynk TRANSPORT]/[Blynk TX]/[Blynk STATUS]/
// [WiFi] verbose lines, leaving only the essential engMsg()/perfReport()
// lines (connection state changes, [PERF], [HEALTH], etc.) — matching the
// original sketch's normal-operation verbosity.
#define DEBUGON        false
// Enable Blynk library diagnostics when either verbose debugging is enabled
// OR the stock BlynkSimpleEsp32 connection mode is being tested.
#if DEBUGON || (CONNECTION_MODE == 1)
  #define BLYNK_PRINT Serial
#endif

// v3.9.113: enable the stock Blynk v0.6.1 protocol diagnostics for this
// heartbeat investigation.  This is deliberately library-side diagnostic
// output, not a second heartbeat implementation.  In particular, when the
// library prints "Heartbeat timeout" with BLYNK_DEBUG enabled it also reports
// its internal timing values, allowing us to compare the actual lastActivityIn
// timestamp against BLYNK_HEARTBEAT + 3*BLYNK_TIMEOUT_MS.
#define BLYNK_DEBUG

// v3.9.22: one master switch for verbose serial diagnostics.
// true  = deep network/Core-1/Blynk diagnostics enabled.
// false = suppress diagnostic chatter; important startup/errors still print.
#if DEBUGON
  void isolatedDebugPrintln(const char* text);
  void isolatedDebugPrintf(const char* fmt, ...);
  #define DBG_PRINTLN(x) isolatedDebugPrintln(x)
  #define DBG_PRINTF(...) isolatedDebugPrintf(__VA_ARGS__)
#else
  #define DBG_PRINTLN(x) do {} while (0)
  #define DBG_PRINTF(...) do {} while (0)
#endif

#define GPS_DEBUG_PRINTOUT false

#include <WiFi.h>

#include <esp_wifi.h>

// Stock transport tuning: keep Blynk I/O bounded to 1s per Blynk.run() call.
// Disconnected attempts are separately throttled below so a failed local-server
// connection cannot make Core 1 spend the whole loop repeatedly reconnecting.

#if CONNECTION_MODE != 1
  #error "v3.9.88 must use CONNECTION_MODE 1 (stock BlynkSimpleEsp32 transport)"
#endif

// v3.9.87: The historical custom transport remains in the source for reference,
// but the active global Blynk object below is the stock BlynkSimpleEsp32 object.
// No raw-lwIP transport is used by the running firmware.
// v3.9.12/13: these were previously included further down (right after the
// Blynk object is constructed), but BoundedSocketClient below now implements
// its transport directly on raw lwIP sockets (lwip_socket/connect/send/recv/
// select/close, SOL_SOCKET, etc.), so the declarations must come before that
// class definition, not after it.
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/etharp.h>
#include <lwip/ip4.h>
#include <lwip/tcpip.h>
#include <lwip/priv/tcp_priv.h>
#include <errno.h>
#include <string.h>

// v3.7 reliability fix: Blynk v0.6.1's stock ESP32 transport calls
// WiFiClient::connect() without an explicit connection timeout. On this
// firmware that can block Core 1 long enough to trigger the ESP32 Interrupt
// WDT immediately after WiFi connects.
//
// Do NOT include BlynkSimpleEsp32.h here. That header creates its own global
// Blynk object using the stock BlynkArduinoClient transport, so replacing it
// from the sketch would cause a duplicate global object and cannot safely
// override the transport. Instead, the small ESP32 wrapper below is reproduced
// from the v0.6.1 header, with ONLY the transport type changed to the bounded
// client. All Blynk protocol/application behaviour remains the same.
#define BLYNK_SEND_ATOMIC
#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <Adapters/BlynkArduinoClient.h>

// v3.9.13 FIX: the previous fix (SO_SNDTIMEO/SO_RCVTIMEO via setsockopt) had
// NO effect — loopMax/FASTmax still froze for 11+ seconds. Root cause: this
// ESP-IDF/lwIP build simply doesn't act on SO_SNDTIMEO/SO_RCVTIMEO for TCP
// sockets (setsockopt() reports success but a blocking send()/recv() can
// still hang indefinitely whenever the peer stops draining/filling its side
// of the connection). Socket-option timeouts cannot be trusted on this
// platform at all. The only bound that has actually been proven to work in
// this file is the raw-socket, non-blocking, select()-gated technique
// blynkProbeStart()/blynkProbePoll() use for the connect probe. BoundedSocketClient
// below applies that exact same technique to the FULL transport — connect,
// read, AND write — by reimplementing the small subset of Arduino's Client
// interface that BlynkArduinoClient actually calls. The socket is put into
// O_NONBLOCK once (at connect time) and every operation is then a
// non-blocking syscall gated by select() with a hard millisecond budget, so
// none of them can ever hang the Core 1 loop() again, regardless of what the
// underlying lwIP build does or doesn't honour at the socket-option level.
// v3.9.20 transport diagnostic counters are defined with the other TX
// diagnostics later in the sketch; declare them here because the transport
// class is defined before that section.
extern volatile uint32_t blynkTransportWriteCalls;
extern volatile uint32_t blynkTransportWriteZero;
extern volatile uint32_t blynkTransportWriteMaxMs;
extern volatile uint32_t blynkTransportWriteLastMs;
extern volatile uint32_t blynkTransportReadCalls;
extern volatile uint32_t blynkTransportReadMaxMs;
extern volatile uint32_t blynkTransportReadLastMs;
extern volatile uint32_t blynkTransportConnectCalls;
extern volatile uint32_t blynkTransportConnectFailures;
extern volatile uint32_t blynkTransportConnectTimeouts;
extern volatile uint32_t blynkTransportConnectLastMs;
extern volatile uint32_t blynkTransportWriteEagain;
extern volatile uint32_t blynkTransportWriteTimeouts;
extern volatile uint32_t blynkTransportSocketErrors;
extern volatile int32_t  blynkTransportLastErrno;
extern volatile int32_t  blynkTransportLastSoError;
extern volatile int32_t  blynkTransportLastSndBuf;
extern volatile uint16_t blynkTransportLastLocalPort;
extern volatile uint16_t blynkTransportLastRemotePort;
extern volatile uint32_t blynkTransportConnectionAgeMs;
extern volatile uint32_t blynkTransportLastFailureAgeMs;
extern volatile bool     blynkTransportKeepaliveConfigured;
extern volatile unsigned long blynkLastRxMillis;
extern unsigned long blynkLastTxAttemptMillis;
extern unsigned long blynkLastTxSuccessMillis;
extern unsigned long blynkLastConnectedMillis;
extern volatile uint32_t perfMaxBlynkRunMs;
extern volatile uint32_t diagBlynkRunSlow;
static uint16_t blynkTxPendingCount();

// v3.9.81 compile-order fix: diagnosticRxTrace() is an inline class method,
// so these globals must be declared before the class definition. Their actual
// storage definitions remain later in the file, preserving the existing logic.
extern uint32_t blynkForcePingCount;
extern unsigned long blynkLastForcePingMillis;

class BoundedBlynkArduinoClient : public BlynkArduinoClient
{
  int rawSock = -1;
  unsigned long rawConnectionStarted = 0;

  static constexpr uint32_t CONNECT_BUDGET_MS = 300UL;
  // v3.9.24: raised from 250UL. This value was tuned to stay far below the
  // ESP32 Interrupt WDT (~300ms) so a hung write() call could never trip
  // it — that goal only requires "bounded," not "tightly bounded." 250ms
  // is well within normal WiFi retransmission/jitter range (a single
  // 802.11 retry or brief airtime contention can easily exceed it), so a
  // healthy connection with a momentary send-buffer stall was being
  // torn down and fully reconnected — confirmed in production logs as
  // "WRITE TIMEOUT after 251ms ... errno=11(EAGAIN)" immediately followed
  // by "Connection lost" and a fresh handshake, even though the TCP
  // session and WiFi link were otherwise fine.
  // v3.9.26: reduced from 1200ms to 750ms. 1200ms was causing Core 1 stalls
  // (SLOWmax=1200ms, Core1Stalls=1) which triggered WiFi driver corruption.
  // 750ms keeps a reasonable safety margin for WiFi jitter while preventing
  // long Core 1 stalls that could destabilize the WiFi driver.
  static constexpr uint32_t WRITE_BUDGET_MS   = 750UL;
  static constexpr uint32_t READ_BUDGET_MS    = BLYNK_TIMEOUT_MS;

  // v3.9.120 FIX: BlynkProtocol::run()'s inner "while (avail ||
  // conn.available() > 0)" loop (stock BlynkProtocol.h) can call
  // processInput() -> readHeader() -> this class's read() an UNBOUNDED
  // number of times within a single Blynk.run() call. Each individual
  // read() was correctly bounded to READ_BUDGET_MS (1000ms), but nothing
  // bounded the *loop*, so a burst of small back-to-back messages (e.g.
  // login RESPONSE + HARDWARE ack + INTERNAL sync) could stack 2-3 near-
  // timeout reads into one 2.8-3.0s Blynk.run() call. That single call is
  // long enough that the server/lwIP side tears the session down, which
  // is the direct cause of the ~20s "Disconnected" -> reconnect cycle seen
  // in the field logs.
  //
  // The 5-byte protocol header is fixed-size and arrives as one small
  // packet on a LAN; it does not need the full 1000ms budget that the
  // (potentially larger, fragmented) message body legitimately does.
  // Giving the header its own short budget stops any single read() call
  // from ever needing more than HEADER_READ_BUDGET_MS, which keeps the
  // cumulative time of run()'s inner loop bounded even when several
  // messages are serviced back-to-back in one pass.
  static constexpr uint32_t HEADER_READ_BUDGET_MS = 120UL;
  static constexpr size_t   PROTOCOL_HEADER_LEN    = 5;

  bool waitForSocket(bool wantWrite, uint32_t timeoutMs)
  {
    if (rawSock < 0) return false;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(rawSock, &fds);

    struct timeval tv;
    tv.tv_sec = timeoutMs / 1000UL;
    tv.tv_usec = (timeoutMs % 1000UL) * 1000UL;

    // v3.9.124 DIAGNOSTIC: lwip_select() is the one remaining blocking
    // primitive in the connect()/write()/read() path that has never been
    // independently timed. Every operation around it is individually
    // bounded well under 3000ms, yet the RUN SPLIT diagnostic shows the
    // 2.7-3.0s block is real work inside run(), not scheduler preemption
    // before run() is entered. If lwip_select() itself returns
    // significantly later than the timeoutMs it was given, that proves
    // the ESP32/lwIP task (or another higher-priority task on Core 1) is
    // preempting this call while it's parked in the kernel — a delay this
    // wrapper cannot distinguish from "select() legitimately wasn't
    // ready" any other way.
    const uint32_t selectStartUs = micros();
    const int rc = wantWrite
      ? lwip_select(rawSock + 1, NULL, &fds, NULL, &tv)
      : lwip_select(rawSock + 1, &fds, NULL, NULL, &tv);
    const uint32_t selectElapsedMs = (uint32_t)((micros() - selectStartUs) / 1000UL);
    if (selectElapsedMs > timeoutMs + 50UL) {
      DBG_PRINTF("[Blynk SELECT SPLIT] requestedMs=%lu actualMs=%lu overshoot=%lums wantWrite=%d (select() blocked well past its own timeout — points at scheduler preemption of this task, not a slow peer)\n",
                 (unsigned long)timeoutMs, (unsigned long)selectElapsedMs,
                 (unsigned long)(selectElapsedMs - timeoutMs), (int)wantWrite);
    }

    return rc > 0 && FD_ISSET(rawSock, &fds);
  }

  void closeRawSocket()
  {
    if (rawSock >= 0) {
      lwip_close(rawSock);
      rawSock = -1;
    }
    isConn = false;
  }

  void printSocketDiagnostics(const char* reason, int failureErrno = -1)
  {
    int soError = 0;
    socklen_t soLen = sizeof(soError);
    int sndBuf = 0;
    socklen_t sndLen = sizeof(sndBuf);
    sockaddr_in localAddr = {};
    sockaddr_in peerAddr = {};
    socklen_t localLen = sizeof(localAddr);
    socklen_t peerLen = sizeof(peerAddr);
    bool soOk = false;
    bool sndOk = false;
    bool localOk = false;
    bool peerOk = false;

    const int savedErrno = (failureErrno >= 0) ? failureErrno : errno;
    int soErrno = 0, sndErrno = 0, localErrno = 0, peerErrno = 0;
    if (rawSock >= 0) {
      int rc = getsockopt(rawSock, SOL_SOCKET, SO_ERROR, &soError, &soLen);
      if (rc == 0) soOk = true; else soErrno = errno;
      rc = getsockopt(rawSock, SOL_SOCKET, SO_SNDBUF, &sndBuf, &sndLen);
      if (rc == 0) sndOk = true; else sndErrno = errno;
      rc = getsockname(rawSock, (sockaddr*)&localAddr, &localLen);
      if (rc == 0) localOk = true; else localErrno = errno;
      rc = getpeername(rawSock, (sockaddr*)&peerAddr, &peerLen);
      if (rc == 0) peerOk = true; else peerErrno = errno;
    }

    blynkTransportLastErrno = savedErrno;
    blynkTransportLastSoError = soOk ? soError : -1;
    blynkTransportLastSndBuf = sndOk ? sndBuf : -1;
    blynkTransportLastLocalPort = localOk ? ntohs(localAddr.sin_port) : 0;
    blynkTransportLastRemotePort = peerOk ? ntohs(peerAddr.sin_port) : 0;
    blynkTransportLastFailureAgeMs = (blynkTransportConnectionAgeMs > 0)
                                   ? blynkTransportConnectionAgeMs : 0;
    if (soOk && soError != 0) ++blynkTransportSocketErrors;

    DBG_PRINTF("[Blynk NET DIAG] reason=%s fd=%d connected=%d writeErrno=%d(%s) SO_ERROR=%d SO_SNDBUF=%d age=%lums localPort=%u remotePort=%u\n",
               reason ? reason : "?", rawSock, isConn ? 1 : 0,
               savedErrno, strerror(savedErrno), soOk ? soError : -1,
               sndOk ? sndBuf : -1,
               (unsigned long)blynkTransportConnectionAgeMs,
               localOk ? (unsigned)ntohs(localAddr.sin_port) : 0U,
               peerOk ? (unsigned)ntohs(peerAddr.sin_port) : 0U);
    DBG_PRINTF("[Blynk NET DIAG] WiFi status=%d RSSI=%ddBm IP=%s GW=%s SSID=%s heap=%u\n",
               (int)WiFi.status(), WiFi.RSSI(), WiFi.localIP().toString().c_str(),
               WiFi.gatewayIP().toString().c_str(), WIFI_SSID,
               (unsigned)ESP.getFreeHeap());
  }

  void configureKeepalive()
  {
    blynkTransportKeepaliveConfigured = false;
    if (rawSock < 0) return;

    int enable = 1;
    if (setsockopt(rawSock, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable)) != 0) {
      DBG_PRINTF("[Blynk RAW] SO_KEEPALIVE failed errno=%d(%s)\n", errno, strerror(errno));
      return;
    }

    // Aggressive enough for diagnostics: a dead LAN path should be detected
    // within seconds, not after the many-hour TCP default. Use only options
    // exposed by the current lwIP headers.
#ifdef TCP_KEEPIDLE
    int idle = 10;
    if (setsockopt(rawSock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle)) != 0)
      DBG_PRINTF("[Blynk RAW] TCP_KEEPIDLE failed errno=%d(%s)\n", errno, strerror(errno));
#endif
#ifdef TCP_KEEPINTVL
    int interval = 3;
    if (setsockopt(rawSock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval)) != 0)
      DBG_PRINTF("[Blynk RAW] TCP_KEEPINTVL failed errno=%d(%s)\n", errno, strerror(errno));
#endif
#ifdef TCP_KEEPCNT
    int count = 3;
    if (setsockopt(rawSock, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count)) != 0)
      DBG_PRINTF("[Blynk RAW] TCP_KEEPCNT failed errno=%d(%s)\n", errno, strerror(errno));
#endif
    blynkTransportKeepaliveConfigured = true;
    DBG_PRINTLN("[Blynk RAW] TCP keepalive enabled (10s idle / 3s interval / 3 probes where supported)");
  }

public:
  // IMPORTANT: do not construct BlynkArduinoClient with a WiFiClient here.
  // BlynkProtocol uses the transport type directly, so this derived class
  // can provide the complete transport without ever entering
  // Arduino-ESP32 NetworkClient::write().
  BoundedBlynkArduinoClient()
      : BlynkArduinoClient() {}

  void disconnect()
  {
    closeRawSocket();
  }

  bool connect()
  {
    ++blynkTransportConnectCalls;
    closeRawSocket();

    IPAddress target;
    if (domain) {
      const uint32_t preLogUs = micros();
      BLYNK_LOG4(BLYNK_F("Connecting to "), domain, ':', port);
      const uint32_t logElapsedMs = (uint32_t)((micros() - preLogUs) / 1000UL);
      if (logElapsedMs > 50UL) {
        DBG_PRINTF("[Blynk SERIAL SPLIT] \"Connecting to\" log line took %lums (Serial.print blocking on a full/undrained TX buffer would show up here)\n",
                   (unsigned long)logElapsedMs);
      }
      if (target.fromString(domain)) {
        // Numeric IP: no DNS involved.
      } else {
        // BLYNK_SERVER is normally a numeric LAN IP.  If a hostname is used,
        // retain the existing Arduino DNS path as a best-effort fallback.
        if (!WiFi.hostByName(domain, target)) {
          DBG_PRINTLN("[Blynk RAW] DNS lookup failed");
          return false;
        }
      }
    } else {
      target = addr;
      BLYNK_LOG_IP("Connecting to ", addr);
    }

    // v3.9.123 DIAGNOSTIC: field logs show the FIRST Blynk.run() call after
    // a fresh WiFi association (state=CONNECTING, no socket ever opened
    // yet) taking ~2.7-3.0s end-to-end, even though every operation below
    // is individually bounded to well under 400ms. The likely cause is
    // lwip_socket()/lwip_connect() themselves blocking on the lwIP task's
    // internal queue/memory pool while it is still processing post-
    // association housekeeping (DHCP lease install, ARP, netif-up) from
    // the WiFi connect that just completed a few hundred ms earlier —
    // neither call is gated by waitForSocket()/O_NONBLOCK, since the
    // socket doesn't exist yet and O_NONBLOCK can only be applied after
    // lwip_socket() returns. Timing them directly here confirms or rules
    // this out instead of inferring it from the outside.
    const uint32_t preSocketUs = micros();
    rawSock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    const uint32_t postSocketUs = micros();
    if (rawSock < 0) {
      ++blynkTransportConnectFailures;
      DBG_PRINTF("[Blynk RAW] socket() failed errno=%d(%s)\n", errno, strerror(errno));
      rawSock = -1;
      return false;
    }
    const uint32_t socketCallMs = (uint32_t)((postSocketUs - preSocketUs) / 1000UL);
    if (socketCallMs > 50UL) {
      DBG_PRINTF("[Blynk RAW CONNECT SPLIT] lwip_socket()=%lums (blocking call, not bounded by waitForSocket — a large value here means the lwIP task itself was busy, most often right after a fresh WiFi association)\n",
                 (unsigned long)socketCallMs);
    }

    // Put the socket into O_NONBLOCK permanently.  This is the critical fix:
    // no later send()/recv() can enter the Arduino NetworkClient retry loop
    // (which can consume ~10 seconds: 10 x 1s select waits).
    const int flags = lwip_fcntl(rawSock, F_GETFL, 0);
    if (flags < 0 || lwip_fcntl(rawSock, F_SETFL, flags | O_NONBLOCK) < 0) {
      ++blynkTransportConnectFailures;
      DBG_PRINTF("[Blynk RAW] fcntl(O_NONBLOCK) failed errno=%d(%s)\n", errno, strerror(errno));
      closeRawSocket();
      return false;
    }

    struct sockaddr_in serverAddr = {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = (uint32_t)target;

    const int rc = lwip_connect(rawSock,
                                (struct sockaddr*)&serverAddr,
                                sizeof(serverAddr));
    if (rc < 0 && errno != EINPROGRESS) {
      DBG_PRINTF("[Blynk RAW] connect() failed errno=%d(%s)\n", errno, strerror(errno));
      ++blynkTransportConnectFailures;
      closeRawSocket();
      return false;
    }

    if (rc < 0) {
      if (!waitForSocket(true, CONNECT_BUDGET_MS)) {
        DBG_PRINTF("[Blynk RAW] connect timeout after %lums\n",
                   (unsigned long)CONNECT_BUDGET_MS);
        ++blynkTransportConnectFailures;
        closeRawSocket();
        return false;
      }

      int sockErr = 0;
      socklen_t sockErrLen = sizeof(sockErr);
      if (getsockopt(rawSock, SOL_SOCKET, SO_ERROR, &sockErr, &sockErrLen) < 0 || sockErr != 0) {
        DBG_PRINTF("[Blynk RAW] connect SO_ERROR=%d errno=%d(%s)\n", sockErr, errno, strerror(errno));
        ++blynkTransportConnectFailures;
        closeRawSocket();
        return false;
      }
    }

    isConn = true;
    rawConnectionStarted = millis();
    blynkTransportConnectionAgeMs = 0;
    configureKeepalive();
    DBG_PRINTF("[Blynk RAW] connected fd=%d target=%s:%u RSSI=%ddBm localIP=%s gateway=%s\n",
               rawSock, target.toString().c_str(), (unsigned)port, WiFi.RSSI(),
               WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str());
    printSocketDiagnostics("CONNECT_OK");
    return true;
  }

  size_t write(const void* buf, size_t len)
  {
    const uint32_t callNo = ++blynkTransportWriteCalls;
    const unsigned long start = millis();
    DBG_PRINTF("[Blynk TRANSPORT WRITE] START call=%lu len=%u connected=%d fd=%d heap=%u\n",
                  (unsigned long)callNo, (unsigned)len,
                  isConn ? 1 : 0, rawSock, (unsigned)ESP.getFreeHeap());

    if (!isConn || rawSock < 0 || !buf || len == 0) {
      ++blynkTransportWriteZero;
      blynkTransportWriteLastMs = millis() - start;
      return 0;
    }

    const uint8_t* p = (const uint8_t*)buf;
    size_t sent = 0;
    const unsigned long deadline = start + WRITE_BUDGET_MS;

    while (sent < len) {
      const unsigned long now = millis();
      if ((long)(deadline - now) <= 0) {
        ++blynkTransportWriteZero;
        ++blynkTransportWriteTimeouts;
        blynkTransportConnectionAgeMs = rawConnectionStarted ? (now - rawConnectionStarted) : 0;
        const int writeErrno = errno;
        DBG_PRINTF("[Blynk RAW] WRITE TIMEOUT after %lums sent=%u/%u errno=%d(%s)\n",
                   (unsigned long)(now - start), (unsigned)sent, (unsigned)len, writeErrno, strerror(writeErrno));
        printSocketDiagnostics("WRITE_DEADLINE", writeErrno);
        closeRawSocket();
        blynkTransportWriteLastMs = now - start;
        if (blynkTransportWriteLastMs > blynkTransportWriteMaxMs)
          blynkTransportWriteMaxMs = blynkTransportWriteLastMs;
        return 0;
      }

      const uint32_t remainingMs = (uint32_t)(deadline - now);
      if (!waitForSocket(true, remainingMs)) {
        const unsigned long elapsed = millis() - start;
        ++blynkTransportWriteZero;
        ++blynkTransportWriteTimeouts;
        blynkTransportConnectionAgeMs = rawConnectionStarted ? (millis() - rawConnectionStarted) : 0;
        const int writeErrno = errno;
        DBG_PRINTF("[Blynk RAW] WRITE TIMEOUT after %lums sent=%u/%u errno=%d(%s)\n",
                   elapsed, (unsigned)sent, (unsigned)len, writeErrno, strerror(writeErrno));
        printSocketDiagnostics("WRITE_SELECT_TIMEOUT", writeErrno);
        closeRawSocket();
        blynkTransportWriteLastMs = elapsed;
        if (elapsed > blynkTransportWriteMaxMs) blynkTransportWriteMaxMs = elapsed;
        return 0;
      }

      const int n = lwip_send(rawSock, p + sent, len - sent, MSG_DONTWAIT);
      if (n > 0) {
        sent += (size_t)n;
        continue;
      }

      if (n == 0) {
        DBG_PRINTF("[Blynk RAW] WRITE returned 0 errno=%d\n", errno);
        closeRawSocket();
        ++blynkTransportWriteZero;
        break;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) ++blynkTransportWriteEagain;
        continue;
      }

      DBG_PRINTF("[Blynk RAW] WRITE error errno=%d\n", errno);
      closeRawSocket();
      ++blynkTransportWriteZero;
      break;
    }

    const unsigned long elapsed = millis() - start;
    blynkTransportWriteLastMs = elapsed;
    if (elapsed > blynkTransportWriteMaxMs) blynkTransportWriteMaxMs = elapsed;

    DBG_PRINTF("[Blynk TRANSPORT WRITE] END   call=%lu result=%u/%u duration=%lums max=%lums connected=%d fd=%d\n",
                  (unsigned long)callNo, (unsigned)sent, (unsigned)len,
                  elapsed, (unsigned long)blynkTransportWriteMaxMs,
                  isConn ? 1 : 0, rawSock);
    return sent == len ? sent : 0;
  }

  // v3.9.82: passive Blynk protocol header capture.
  // Blynk uses a 5-byte header: command(1), messageId(2), body/status(2).
  // This parser observes bytes already returned by lwip_recv(); it does not
  // perform any additional socket reads and therefore cannot consume data.
  uint8_t  rxHeaderCapture[5] = {0, 0, 0, 0, 0};
  uint8_t  rxHeaderCapturePos = 0;
  uint16_t rxHeaderCaptureBodyRemaining = 0;
  bool     rxHeaderCaptureInBody = false;
  uint32_t rxHeaderCaptureCount = 0;
  uint32_t rxHeaderCapturePingCount = 0;
  uint32_t rxHeaderCaptureResponseCount = 0;
  uint32_t rxHeaderCaptureHardwareCount = 0;
  uint32_t rxHeaderCaptureInternalCount = 0;
  uint32_t rxHeaderCaptureOtherCount = 0;
  uint8_t  rxHeaderCaptureLastCmd = 0;
  uint16_t rxHeaderCaptureLastMsgId = 0;
  uint16_t rxHeaderCaptureLastBodyLen = 0;
  uint32_t rxHeaderCaptureLastMillis = 0;
  uint32_t rxHeaderCaptureSequence = 0;
  char rxHeaderCaptureLastHex[16] = "-- -- -- -- --";

  void captureRxBytes(const uint8_t* data, size_t len)
  {
    if (!data || len == 0) return;

    for (size_t i = 0; i < len; ++i) {
      const uint8_t b = data[i];

      if (rxHeaderCaptureInBody) {
        if (rxHeaderCaptureBodyRemaining > 0) {
          --rxHeaderCaptureBodyRemaining;
        }
        if (rxHeaderCaptureBodyRemaining == 0) {
          rxHeaderCaptureInBody = false;
          rxHeaderCapturePos = 0;
        }
        continue;
      }

      rxHeaderCapture[rxHeaderCapturePos++] = b;
      if (rxHeaderCapturePos < 5) continue;

      const uint8_t cmd = rxHeaderCapture[0];
      const uint16_t msgId = ((uint16_t)rxHeaderCapture[1] << 8) |
                             (uint16_t)rxHeaderCapture[2];
      const uint16_t bodyLenOrStatus = ((uint16_t)rxHeaderCapture[3] << 8) |
                                      (uint16_t)rxHeaderCapture[4];
      // For command 0 (RESPONSE), bytes 4-5 are a response/status code, not
      // a body length. RESPONSE messages therefore have no body to skip here.
      const uint16_t bodyLen = (cmd == 0) ? 0 : bodyLenOrStatus;
      const uint32_t now = millis();

      ++rxHeaderCaptureCount;
      ++rxHeaderCaptureSequence;
      snprintf(rxHeaderCaptureLastHex, sizeof(rxHeaderCaptureLastHex),
               "%02X %02X %02X %02X %02X",
               rxHeaderCapture[0], rxHeaderCapture[1], rxHeaderCapture[2],
               rxHeaderCapture[3], rxHeaderCapture[4]);
      rxHeaderCaptureLastCmd = cmd;
      rxHeaderCaptureLastMsgId = msgId;
      rxHeaderCaptureLastBodyLen = bodyLen;
      rxHeaderCaptureLastMillis = now;

      if (cmd == 6) {
        ++rxHeaderCapturePingCount;
      } else if (cmd == 0) {
        ++rxHeaderCaptureResponseCount;
      } else if (cmd == 20) {
        ++rxHeaderCaptureHardwareCount;
      } else if (cmd == 17) {
        ++rxHeaderCaptureInternalCount;
      } else {
        ++rxHeaderCaptureOtherCount;
      }

      Serial.printf("[Blynk RX HEADER #%lu] cmd=%u msgId=%u field=%u bodyLen=%u hex=%s ping=%lu rsp=%lu hw=%lu int=%lu other=%lu\n",
                    (unsigned long)rxHeaderCaptureSequence,
                    (unsigned)cmd, (unsigned)msgId, (unsigned)bodyLenOrStatus,
                    (unsigned)bodyLen, rxHeaderCaptureLastHex,
                    (unsigned long)rxHeaderCapturePingCount,
                    (unsigned long)rxHeaderCaptureResponseCount,
                    (unsigned long)rxHeaderCaptureHardwareCount,
                    (unsigned long)rxHeaderCaptureInternalCount,
                    (unsigned long)rxHeaderCaptureOtherCount);

      // If this message has a body, skip it until the next header. A zero-body
      // message is complete immediately and the next byte starts a new header.
      rxHeaderCaptureBodyRemaining = bodyLen;
      rxHeaderCapturePos = 0;
      rxHeaderCaptureInBody = (bodyLen != 0);
    }
  }

  size_t read(void* buf, size_t len)
  {
    const uint32_t callNo = ++blynkTransportReadCalls;
    const unsigned long start = millis();
    DBG_PRINTF("[Blynk TRANSPORT READ]  START call=%lu len=%u connected=%d fd=%d\n",
                  (unsigned long)callNo, (unsigned)len,
                  isConn ? 1 : 0, rawSock);

    if (!isConn || rawSock < 0 || !buf || len == 0) return 0;

    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;
    // v3.9.120: a request for exactly the 5-byte protocol header gets a
    // short budget instead of the full READ_BUDGET_MS. This is what
    // prevents a burst of back-to-back small messages from stacking
    // multiple near-1000ms waits inside one Blynk.run() call. Body reads
    // (len != 5) keep the existing, larger budget.
    const uint32_t effectiveBudgetMs =
        (len == PROTOCOL_HEADER_LEN) ? HEADER_READ_BUDGET_MS : READ_BUDGET_MS;
    const unsigned long deadline = start + effectiveBudgetMs;

    while (got < len) {
      const unsigned long now = millis();
      if ((long)(deadline - now) <= 0) {
        DBG_PRINTF("[Blynk RAW] READ TIMEOUT after %lums got=%u/%u\n",
                      (unsigned long)(now - start), (unsigned)got, (unsigned)len);
        closeRawSocket();
        break;
      }

      const uint32_t remainingMs = (uint32_t)(deadline - now);
      if (!waitForSocket(false, remainingMs)) {
        DBG_PRINTF("[Blynk RAW] READ TIMEOUT after %lums got=%u/%u errno=%d\n",
                      (unsigned long)(millis() - start), (unsigned)got, (unsigned)len, errno);
        closeRawSocket();
        break;
      }

      const int n = lwip_recv(rawSock, p + got, len - got, MSG_DONTWAIT);
      ++rxRecvCalls;
      rxRecvLastMillis = millis();
      if (n > 0) {
        ++rxRecvDataCalls;
        rxRecvBytes += (uint32_t)n;
        rxRecvLastBytes = (uint32_t)n;
        if ((uint32_t)n > rxRecvMaxBytes) rxRecvMaxBytes = (uint32_t)n;
        captureRxBytes(p + got, (size_t)n);
        got += (size_t)n;
        blynkLastRxMillis = rxRecvLastMillis;
        continue;
      }
      if (n == 0) {
        ++rxRecvZeroCalls;
        DBG_PRINTLN("[Blynk RAW] peer closed during READ");
        closeRawSocket();
        break;
      }
      rxRecvLastErrno = errno;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        ++rxRecvEagainCalls;
        continue;
      }
      if (errno == EINTR) {
        ++rxRecvEintrCalls;
        continue;
      }
      ++rxRecvErrorCalls;
      DBG_PRINTF("[Blynk RAW] READ error errno=%d\n", errno);
      closeRawSocket();
      break;
    }

    const unsigned long elapsed = millis() - start;
    blynkTransportReadLastMs = elapsed;
    if (elapsed > blynkTransportReadMaxMs) blynkTransportReadMaxMs = elapsed;

    DBG_PRINTF("[Blynk TRANSPORT READ]  END   call=%lu result=%u/%u duration=%lums max=%lums connected=%d fd=%d\n",
                  (unsigned long)callNo, (unsigned)got, (unsigned)len,
                  elapsed, (unsigned long)blynkTransportReadMaxMs,
                  isConn ? 1 : 0, rawSock);
    return got;
  }

  bool connected()
  {
    if (!isConn || rawSock < 0) return false;
    blynkTransportConnectionAgeMs = rawConnectionStarted ? (millis() - rawConnectionStarted) : 0;

    ++connectedChecks;
    uint8_t dummy = 0;
    errno = 0;
    const int n = lwip_recv(rawSock, &dummy, 1, MSG_DONTWAIT | MSG_PEEK);
    connectedPeekLastMillis = millis();
    connectedPeekLastErrno = (n < 0) ? errno : 0;
    if (n > 0) {
      ++connectedPeekData;
      return true;
    }
    if (n == 0) {
      // v3.9.24: log before tearing down — this path previously closed the
      // socket completely silently, which is why earlier production drops
      // showed "Connection lost" with no preceding transport-layer message
      // at all. n==0 on a peek means the peer sent a TCP FIN (orderly
      // close), so this branch is a genuine, correctly-detected disconnect.
      ++connectedPeekZero;
      DBG_PRINTLN("[Blynk RAW] connected(): peer closed (FIN) on MSG_PEEK — treating as disconnected");
      closeRawSocket();
      return false;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ++connectedPeekEagain;
      return true;
    }
    if (errno == EINTR) {
      ++connectedPeekEintr;
      return true;
    }

    // v3.9.24: previously, ANY other errno here silently closed the socket
    // and reported "not connected" with zero diagnostic output — this was
    // very likely the actual cause of unexplained "Connection lost" events
    // that showed no preceding WRITE/READ TIMEOUT or "peer closed" message
    // in the log. A single MSG_PEEK returning an unexpected errno is not
    // proof the connection is dead (transient errno values can occur
    // during normal TCP retransmission/congestion windows on WiFi); only
    // a small number of errno values genuinely indicate the socket is
    // unusable. Log every occurrence so future drops are diagnosable, and
    // only treat the connection as dead on errno values that actually mean
    // the socket is broken.
    const int peekErrno = errno;
    switch (peekErrno) {
      case ECONNRESET:
      case ENOTCONN:
      case EPIPE:
      case EBADF:
        ++connectedPeekFatal;
        DBG_PRINTF("[Blynk RAW] connected(): fatal errno=%d(%s) on MSG_PEEK — socket is dead\n",
                      peekErrno, strerror(peekErrno));
        closeRawSocket();
        return false;
      default:
        // Anything else observed here (log it so we can classify it if it
        // recurs) is treated as transient rather than an automatic kill —
        // report still-connected and let the normal read()/write() paths,
        // which already have their own bounded timeouts, be the ones that
        // actually detect a genuinely dead connection.
        DBG_PRINTF("[Blynk RAW] connected(): non-fatal errno=%d(%s) on MSG_PEEK — treating as still connected\n",
                      peekErrno, strerror(peekErrno));
        return true;
    }
  }

  int available()
  {
    if (!isConn || rawSock < 0) return 0;

    int count = 0;
    if (lwip_ioctl(rawSock, FIONREAD, &count) < 0) {
      DBG_PRINTF("[Blynk RAW] FIONREAD failed errno=%d\n", errno);
      closeRawSocket();
      return 0;
    }
    // v3.9.128 DIAGNOSTIC: FIONREAD is the true kernel socket receive-buffer
    // byte count — data the NIC/lwIP stack has already accepted, independent
    // of whether the application has read it yet. Field reports say a
    // previous test showed the server's heartbeat reaching the ESP32 but
    // being "lost in some layer" before surfacing as a processed RESPONSE.
    // Every existing diagnostic in this file (RX HEADER, RX OBSERVATION)
    // only fires AFTER a successful read()/readHeader() — there was no
    // logging at the FIONREAD boundary itself, so a byte count reported
    // here that never turns into a subsequent read()/captureRxBytes() hit
    // would be direct proof of data arriving at the socket and then being
    // lost between here and being consumed — something in this class's own
    // read()/run()-calling code, not the network or the server. Logging
    // every nonzero count (rate-limited) lets that gap be seen directly
    // instead of inferred.
    if (count > 0) {
      ++availableNonzeroCount;
      availableLastNonzeroMillis = millis();
      availableLastNonzeroBytes = (uint32_t)count;
      const unsigned long nowMs = millis();
      if (nowMs - availableLastLogMillis >= 200UL) {
        availableLastLogMillis = nowMs;
        DBG_PRINTF("[Blynk RAW AVAILABLE] FIONREAD=%d bytes waiting in kernel socket buffer (rxRecvCalls=%lu rxHeaderCaptureCount=%lu — if these stop advancing while FIONREAD keeps reporting >0, data is arriving but not being consumed)\n",
                     count, (unsigned long)rxRecvCalls, (unsigned long)rxHeaderCaptureCount);
      }
    }
    return count;
  }

  // v3.9.128: FIONREAD/available() ground-truth accounting — see comment above.
  uint32_t availableNonzeroCount = 0;
  uint32_t availableLastNonzeroMillis = 0;
  uint32_t availableLastNonzeroBytes = 0;
  uint32_t availableLastLogMillis = 0;

  // v3.9.81: independent raw TCP RX accounting.
  uint32_t rxRecvCalls = 0;
  uint32_t rxRecvDataCalls = 0;
  uint32_t rxRecvZeroCalls = 0;
  uint32_t rxRecvEagainCalls = 0;
  uint32_t rxRecvEintrCalls = 0;
  uint32_t rxRecvErrorCalls = 0;
  uint64_t rxRecvBytes = 0;
  uint32_t rxRecvLastBytes = 0;
  uint32_t rxRecvMaxBytes = 0;
  uint32_t rxRecvLastMillis = 0;
  int rxRecvLastErrno = 0;

  // v3.9.81: passive connected()/MSG_PEEK accounting.
  uint32_t connectedChecks = 0;
  uint32_t connectedPeekData = 0;
  uint32_t connectedPeekEagain = 0;
  uint32_t connectedPeekEintr = 0;
  uint32_t connectedPeekZero = 0;
  uint32_t connectedPeekFatal = 0;
  uint32_t connectedPeekLastMillis = 0;
  int connectedPeekLastErrno = 0;
  uint64_t rxTraceLastBytes = 0;
  uint32_t rxTraceLastRecvCalls = 0;
  uint32_t rxTraceLastMillis = 0;

  // v3.9.67 DIAGNOSTIC ONLY — inspect the live TCP receive state without
  // consuming any Blynk protocol bytes. This is deliberately passive: no
  // socket close, no Blynk state change, no WiFi action, and no timeout.
  // select() is called with a zero timeout and MSG_PEEK is used for recv().
  void diagnosticRawRxState()
  {
    const uint32_t now = millis();
    if (!isConn || rawSock < 0) {
      Serial.printf("[Blynk RAW RX] fd=%d connected=%d state=NO_SOCKET age=%lums\n",
                    rawSock, isConn ? 1 : 0,
                    (unsigned long)(rawConnectionStarted ? now - rawConnectionStarted : 0));
      return;
    }

    int soError = 0;
    socklen_t soLen = sizeof(soError);
    const int soRc = getsockopt(rawSock, SOL_SOCKET, SO_ERROR, &soError, &soLen);
    const int soErrno = (soRc == 0) ? 0 : errno;

    int rcvBuf = 0;
    socklen_t rcvLen = sizeof(rcvBuf);
    const int rcvRc = getsockopt(rawSock, SOL_SOCKET, SO_RCVBUF, &rcvBuf, &rcvLen);
    const int rcvErrno = (rcvRc == 0) ? 0 : errno;

    int sndBuf = 0;
    socklen_t sndLen = sizeof(sndBuf);
    const int sndRc = getsockopt(rawSock, SOL_SOCKET, SO_SNDBUF, &sndBuf, &sndLen);
    const int sndErrno = (sndRc == 0) ? 0 : errno;

    int keepalive = 0;
    socklen_t keepLen = sizeof(keepalive);
    const int keepRc = getsockopt(rawSock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, &keepLen);
    const int keepErrno = (keepRc == 0) ? 0 : errno;

    int queuedRx = -1;
    const int fionreadRc = lwip_ioctl(rawSock, FIONREAD, &queuedRx);
    const int fionreadErrno = (fionreadRc == 0) ? 0 : errno;

    fd_set readSet, writeSet, exceptSet;
    FD_ZERO(&readSet);
    FD_ZERO(&writeSet);
    FD_ZERO(&exceptSet);
    FD_SET(rawSock, &readSet);
    FD_SET(rawSock, &writeSet);
    FD_SET(rawSock, &exceptSet);
    struct timeval zeroTv;
    zeroTv.tv_sec = 0;
    zeroTv.tv_usec = 0;
    const int selectRc = lwip_select(rawSock + 1, &readSet, &writeSet, &exceptSet, &zeroTv);
    const int selectErrno = (selectRc < 0) ? errno : 0;
    const bool readReady = (selectRc > 0) && FD_ISSET(rawSock, &readSet);
    const bool writeReady = (selectRc > 0) && FD_ISSET(rawSock, &writeSet);
    const bool exceptReady = (selectRc > 0) && FD_ISSET(rawSock, &exceptSet);

    uint8_t dummy = 0;
    errno = 0;
    const int peek = lwip_recv(rawSock, &dummy, 1, MSG_DONTWAIT | MSG_PEEK);
    const int peekErrno = (peek < 0) ? errno : 0;

    const uint32_t age = rawConnectionStarted ? (uint32_t)(now - rawConnectionStarted) : 0;
    blynkTransportConnectionAgeMs = age;

    Serial.printf("[Blynk RAW RX] fd=%d connected=%d age=%lums select=%d R=%d W=%d X=%d selErr=%d SO_ERROR=%d soRc=%d soErrno=%d peek=%d peekErrno=%d queuedRx=%d fionRc=%d fionErrno=%d rcvBuf=%d rcvErrno=%d sndBuf=%d sndErrno=%d keepalive=%d keepRc=%d keepErrno=%d\n",
                  rawSock, isConn ? 1 : 0, (unsigned long)age,
                  selectRc, readReady ? 1 : 0, writeReady ? 1 : 0, exceptReady ? 1 : 0,
                  selectErrno, soError, soRc, soErrno, peek, peekErrno,
                  queuedRx, fionreadRc, fionreadErrno, rcvBuf, rcvErrno,
                  sndBuf, sndErrno, keepalive, keepRc, keepErrno);
  }

  // v3.9.81: periodic passive RX trace. It never consumes socket data.
  void diagnosticRxTrace(const char *reason)
  {
    const uint32_t now = millis();
    int queuedRx = -1;
    int soError = 0;
    int peek = -1;
    int peekErrno = 0;
    if (isConn && rawSock >= 0) {
      errno = 0;
      lwip_ioctl(rawSock, FIONREAD, &queuedRx);
      socklen_t soLen = sizeof(soError);
      getsockopt(rawSock, SOL_SOCKET, SO_ERROR, &soError, &soLen);
      uint8_t dummy = 0;
      errno = 0;
      peek = lwip_recv(rawSock, &dummy, 1, MSG_DONTWAIT | MSG_PEEK);
      peekErrno = (peek < 0) ? errno : 0;
    }
    const uint64_t deltaBytes = rxRecvBytes - rxTraceLastBytes;
    const uint32_t deltaCalls = rxRecvCalls - rxTraceLastRecvCalls;
    const uint32_t traceAge = rxTraceLastMillis ? now - rxTraceLastMillis : 0;
    Serial.printf("[Blynk RX TRACE] reason=%s fd=%d conn=%d sockAge=%lums queuedRx=%d peek=%d peekErr=%d SOERR=%d recvCalls=%lu recvData=%lu recvBytes=%llu dCalls=%lu dBytes=%llu eagain=%lu eintr=%lu zero=%lu errors=%lu lastRecvAge=%lums lastRecvBytes=%lu maxRecv=%lu connChecks=%lu peekData=%lu peekEagain=%lu peekEintr=%lu peekZero=%lu peekFatal=%lu blynkRunMax=%lums blynkRunSlow=%lu blynkRunCalls=%lu txCalls=%lu txLastAge=%lums txSuccessAge=%lums txPending=%u hbCount=%lu hbLastAge=%lums rxHdrCount=%lu rxPing=%lu rxRsp=%lu rxHw=%lu rxInt=%lu rxOther=%lu rxLastCmd=%u rxLastId=%u rxLastLen=%u rxHdrSeq=%lu rxHdrHex=%s rxHdrAge=%lums wifi=%d rssi=%d traceAge=%lums\n",
                  reason ? reason : "UNKNOWN", rawSock, isConn ? 1 : 0,
                  (unsigned long)(rawConnectionStarted ? now - rawConnectionStarted : 0),
                  queuedRx, peek, peekErrno, soError,
                  (unsigned long)rxRecvCalls, (unsigned long)rxRecvDataCalls,
                  (unsigned long long)rxRecvBytes, (unsigned long)deltaCalls,
                  (unsigned long long)deltaBytes, (unsigned long)rxRecvEagainCalls,
                  (unsigned long)rxRecvEintrCalls, (unsigned long)rxRecvZeroCalls,
                  (unsigned long)rxRecvErrorCalls,
                  (unsigned long)(rxRecvLastMillis ? now - rxRecvLastMillis : 0),
                  (unsigned long)rxRecvLastBytes, (unsigned long)rxRecvMaxBytes,
                  (unsigned long)connectedChecks, (unsigned long)connectedPeekData,
                  (unsigned long)connectedPeekEagain, (unsigned long)connectedPeekEintr,
                  (unsigned long)connectedPeekZero, (unsigned long)connectedPeekFatal,
                  (unsigned long)perfMaxBlynkRunMs, (unsigned long)diagBlynkRunSlow,
                  (unsigned long)diagBlynkRunCalls, (unsigned long)blynkTransportWriteCalls,
                  (unsigned long)(blynkLastTxAttemptMillis ? now - blynkLastTxAttemptMillis : 0),
                  (unsigned long)(blynkLastTxSuccessMillis ? now - blynkLastTxSuccessMillis : 0),
                  (unsigned)blynkTxPendingCount(), (unsigned long)blynkForcePingCount,
                  (unsigned long)(blynkLastForcePingMillis ? now - blynkLastForcePingMillis : 0),
                  (unsigned long)rxHeaderCaptureCount,
                  (unsigned long)rxHeaderCapturePingCount,
                  (unsigned long)rxHeaderCaptureResponseCount,
                  (unsigned long)rxHeaderCaptureHardwareCount,
                  (unsigned long)rxHeaderCaptureInternalCount,
                  (unsigned long)rxHeaderCaptureOtherCount,
                  (unsigned)rxHeaderCaptureLastCmd,
                  (unsigned)rxHeaderCaptureLastMsgId,
                  (unsigned)rxHeaderCaptureLastBodyLen,
                  (unsigned long)rxHeaderCaptureSequence,
                  rxHeaderCaptureLastHex,
                  (unsigned long)(rxHeaderCaptureLastMillis ? now - rxHeaderCaptureLastMillis : 0),
                  (int)WiFi.status(), WiFi.RSSI(), (unsigned long)traceAge);
    rxTraceLastBytes = rxRecvBytes;
    rxTraceLastRecvCalls = rxRecvCalls;
    rxTraceLastMillis = now;
  }
};


class BlynkWifiBounded
    : public BlynkProtocol<BoundedBlynkArduinoClient>
{
    typedef BlynkProtocol<BoundedBlynkArduinoClient> Base;
public:
    BlynkWifiBounded(BoundedBlynkArduinoClient& transp)
        : Base(transp)
    {}

    void connectWiFi(const char* ssid, const char* pass)
    {
        BLYNK_LOG2(BLYNK_F("Connecting to "), ssid);
        WiFi.mode(WIFI_STA);
        if (pass && strlen(pass)) {
            WiFi.begin(ssid, pass);
        } else {
            WiFi.begin(ssid);
        }
        while (WiFi.status() != WL_CONNECTED) {
            BlynkDelay(500);
        }
        BLYNK_LOG1(BLYNK_F("Connected to WiFi"));

        IPAddress myip = WiFi.localIP();
        BLYNK_LOG_IP("IP: ", myip);
    }

    void config(const char* auth,
                const char* domain = BLYNK_DEFAULT_DOMAIN,
                uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        this->conn.begin(domain, port);
    }

    void config(const char* auth,
                IPAddress   ip,
                uint16_t    port = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        this->conn.begin(ip, port);
    }

    void begin(const char* auth,
               const char* ssid,
               const char* pass,
               const char* domain = BLYNK_DEFAULT_DOMAIN,
               uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        connectWiFi(ssid, pass);
        config(auth, domain, port);
        while(this->connect() != true) {}
    }

    void begin(const char* auth,
               const char* ssid,
               const char* pass,
               IPAddress   ip,
               uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        connectWiFi(ssid, pass);
        config(auth, ip, port);
        while(this->connect() != true) {}
    }
};

#if CONNECTION_MODE == 0
  static BoundedBlynkArduinoClient _boundedBlynkTransport;
  BlynkWifiBounded Blynk(_boundedBlynkTransport);
#else
  // v3.9.148: retain the stock BlynkArduinoClient/BlynkProtocol path, but
  // override the one defective primitive in Arduino-ESP32 3.3.12. Its stock
  // NetworkClient::write() always permits 100 x 100ms EAGAIN retries, so
  // setTimeout() cannot prevent the measured 10012ms block. This subclass
  // uses the same socket and nonblocking send, with one absolute 750ms budget
  // for the complete Blynk frame. connect/read/available/connected/stop remain
  // the normal WiFiClient implementations.
  #if 0  // v3.9.162: superseded 750ms close-on-backpressure implementation
  class BoundedWriteWiFiClient : public WiFiClient {
  public:
    using WiFiClient::write;

    // BlynkArduinoClient calls the two-argument overload. Route it to the
    // ESP32 core's explicit-timeout implementation so the real Blynk socket
    // can be retried directly without a separate preflight TCP connection.
    int connect(IPAddress ip, uint16_t port) {
      return WiFiClient::connect(ip, port, (int32_t)BLYNK_STOCK_CONNECT_BUDGET_MS);
    }

    int connect(const char *host, uint16_t port) {
      return WiFiClient::connect(host, port, (int32_t)BLYNK_STOCK_CONNECT_BUDGET_MS);
    }

    size_t write(const uint8_t *buf, size_t size) override {
      const int socketFd = fd();
      if (socketFd < 0 || !connected() || !buf || size == 0) return 0;

      const uint32_t started = millis();
      size_t sent = 0;
      while (sent < size) {
        const uint32_t elapsed = millis() - started;
        if (elapsed >= BLYNK_STOCK_WRITE_BUDGET_MS) break;

        const uint32_t remainingMs = BLYNK_STOCK_WRITE_BUDGET_MS - elapsed;
        fd_set writeSet;
        FD_ZERO(&writeSet);
        FD_SET(socketFd, &writeSet);
        struct timeval tv;
        tv.tv_sec = remainingMs / 1000UL;
        tv.tv_usec = (remainingMs % 1000UL) * 1000UL;

        const int ready = lwip_select(socketFd + 1, NULL, &writeSet, NULL, &tv);
        if (ready < 0) {
          if (errno == EINTR) continue;
          blynkTransportLastErrno = errno;
          ++blynkTransportSocketErrors;
          break;
        }
        if (ready == 0) {
          blynkTransportLastErrno = ETIMEDOUT;
          ++blynkTransportWriteTimeouts;
          break;
        }

        const int rc = lwip_send(socketFd, buf + sent, size - sent, MSG_DONTWAIT);
        if (rc > 0) {
          sent += (size_t)rc;
          continue;
        }
        if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) ++blynkTransportWriteEagain;
          continue;
        }
        blynkTransportLastErrno = rc < 0 ? errno : ECONNRESET;
        ++blynkTransportSocketErrors;
        break;
      }

      const uint32_t writeMs = millis() - started;
      blynkTransportWriteLastMs = writeMs;
      if (writeMs > blynkTransportWriteMaxMs) blynkTransportWriteMaxMs = writeMs;
      ++blynkTransportWriteCalls;

      if (sent != size) {
        ++blynkTransportWriteZero;
        blynkTransportConnectionAgeMs = blynkLastConnectedMillis
            ? (millis() - blynkLastConnectedMillis) : 0;
        blynkTransportLastFailureAgeMs = blynkTransportConnectionAgeMs;
        Serial.printf("[Blynk WRITE BOUND] sent=%u/%u elapsed=%lums errno=%ld — closing failed TCP socket\n",
                      (unsigned)sent, (unsigned)size, (unsigned long)writeMs,
                      (long)blynkTransportLastErrno);
        stop();
      }
      return sent;
    }
  };

  #endif

  class BoundedWriteWiFiClient : public WiFiClient {
  public:
    using WiFiClient::write;

    BoundedWriteWiFiClient()
      : deferredHead_(0), deferredCount_(0), deferredSince_(0),
        deferredEvents_(0), deferredDrains_(0), recoverySince_(0),
        awaitingPeerConfirmation_(false), peerProbeArmed_(false),
        peerProbeMsgId_(0) {}

    int connect(IPAddress ip, uint16_t port) {
      clearDeferred();
      ++blynkTransportConnectCalls;
      const uint32_t started = millis();
      const int rc = WiFiClient::connect(
          ip, port, (int32_t)BLYNK_STOCK_CONNECT_BUDGET_MS);
      const uint32_t elapsed = millis() - started;
      blynkTransportConnectLastMs = elapsed;
      if (rc != 1) {
        ++blynkTransportConnectFailures;
        // NetworkClient::connect() waits in select() for the complete budget
        // when no SYN result returns. An immediate RST/refusal returns much
        // sooner and must not be used as evidence that the STA RX path is dead.
        if (elapsed + 50UL >= BLYNK_STOCK_CONNECT_BUDGET_MS) {
          ++blynkTransportConnectTimeouts;
        }
        Serial.printf("[Blynk TCP CONNECT FAIL] elapsed=%lums budget=%lums timeoutClass=%d errno=%d WiFi=%d RSSI=%d failures=%lu timeouts=%lu\n",
                      (unsigned long)elapsed,
                      (unsigned long)BLYNK_STOCK_CONNECT_BUDGET_MS,
                      elapsed + 50UL >= BLYNK_STOCK_CONNECT_BUDGET_MS ? 1 : 0,
                      errno, (int)WiFi.status(), WiFi.RSSI(),
                      (unsigned long)blynkTransportConnectFailures,
                      (unsigned long)blynkTransportConnectTimeouts);
      }
      return rc;
    }

    int connect(const char *host, uint16_t port) {
      clearDeferred();
      ++blynkTransportConnectCalls;
      const uint32_t started = millis();
      const int rc = WiFiClient::connect(
          host, port, (int32_t)BLYNK_STOCK_CONNECT_BUDGET_MS);
      const uint32_t elapsed = millis() - started;
      blynkTransportConnectLastMs = elapsed;
      if (rc != 1) {
        ++blynkTransportConnectFailures;
        if (elapsed + 50UL >= BLYNK_STOCK_CONNECT_BUDGET_MS) {
          ++blynkTransportConnectTimeouts;
        }
        Serial.printf("[Blynk TCP CONNECT FAIL] elapsed=%lums budget=%lums timeoutClass=%d errno=%d WiFi=%d RSSI=%d failures=%lu timeouts=%lu\n",
                      (unsigned long)elapsed,
                      (unsigned long)BLYNK_STOCK_CONNECT_BUDGET_MS,
                      elapsed + 50UL >= BLYNK_STOCK_CONNECT_BUDGET_MS ? 1 : 0,
                      errno, (int)WiFi.status(), WiFi.RSSI(),
                      (unsigned long)blynkTransportConnectFailures,
                      (unsigned long)blynkTransportConnectTimeouts);
      }
      return rc;
    }

    size_t write(const uint8_t *buf, size_t size) override {
      const int socketFd = fd();
      if (socketFd < 0 || !connected() || !buf || size == 0) return 0;

      serviceDeferred();
      size_t sent = 0;
      if (deferredCount_ == 0) {
        const int rc = lwip_send(socketFd, buf, size, MSG_DONTWAIT);
        if (rc > 0) {
          sent = (size_t)rc;
        } else if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) ++blynkTransportWriteEagain;
        } else {
          blynkTransportLastErrno = rc < 0 ? errno : ECONNRESET;
          ++blynkTransportSocketErrors;
          WiFiClient::stop();
          clearDeferred();
          return 0;
        }
      }

      const size_t remainder = size - sent;
      if (remainder && !appendDeferred(buf + sent, remainder)) {
        blynkTransportLastErrno = ENOBUFS;
        ++blynkTransportWriteZero;
        Serial.printf("[Blynk DEFERRED TX] overflow pending=%u add=%u; closing only Blynk socket\n",
                      (unsigned)deferredCount_, (unsigned)remainder);
        WiFiClient::stop();
        clearDeferred();
        return 0;
      }

      blynkTransportWriteLastMs = 0;
      ++blynkTransportWriteCalls;
      // Blynk sees success because every byte is owned by lwIP or this ordered
      // buffer. Application telemetry is gated until pendingBytes() is zero.
      return size;
    }

    void stop() override {
      WiFiClient::stop();
      clearDeferred();
    }

    size_t pendingBytes() const { return deferredCount_; }
    uint32_t pendingAgeMs() const {
      return deferredCount_ && deferredSince_ ? millis() - deferredSince_ : 0;
    }
    uint32_t deferredEvents() const { return deferredEvents_; }
    uint32_t deferredDrains() const { return deferredDrains_; }
    bool awaitingPeerConfirmation() const { return awaitingPeerConfirmation_; }
    uint32_t recoveryAgeMs() const {
      return awaitingPeerConfirmation_ && recoverySince_
          ? millis() - recoverySince_ : 0;
    }

    // Called immediately after the explicit protocol PING is handed to this
    // transport. If backpressure is active, this exact message ID becomes the
    // end-to-end recovery probe. TCP ordering guarantees the server cannot
    // answer it until all older deferred bytes have crossed the missing hole.
    void armPeerProbe(uint16_t msgId) {
      if (!awaitingPeerConfirmation_) return;
      peerProbeMsgId_ = msgId;
      peerProbeArmed_ = true;
      Serial.printf("[Blynk PATH RECOVERY] probe armed msgId=%u pending=%u\n",
                    (unsigned)msgId, (unsigned)deferredCount_);
    }

    // Called by the passive header observer before BlynkProtocol consumes the
    // RESPONSE. Never release on an unrelated/old response or merely because
    // the local queue is empty.
    bool confirmPeerProgress(uint16_t msgId) {
      if (!awaitingPeerConfirmation_ || deferredCount_ != 0) return false;
      // BlynkProtocol automatically sends a PING when transport pending changes
      // from true to false. That PING does not pass through the sketch's
      // explicit heartbeat helper, so it has no peerProbeMsgId_. The first new
      // RESPONSE after the ordered queue drained is therefore the proof. If an
      // explicit probe is armed, retain exact message-ID correlation.
      if (peerProbeArmed_ && msgId != peerProbeMsgId_) return false;
      awaitingPeerConfirmation_ = false;
      peerProbeArmed_ = false;
      recoverySince_ = 0;
      Serial.printf("[Blynk PATH RECOVERY] CONFIRMED msgId=%u; application TX released\n",
                    (unsigned)msgId);
      return true;
    }

    void serviceDeferred() {
      if (deferredCount_ == 0) return;
      const int socketFd = fd();
      if (socketFd < 0 || !WiFiClient::connected()) return;

      // One non-blocking send per loop pass leaves Core 0 available for RX,
      // heartbeat, WiFi and lwIP while TCP retransmission repairs packet loss.
      const size_t contiguous = min(deferredCount_,
          (size_t)BLYNK_STOCK_DEFERRED_TX_BYTES - deferredHead_);
      const int rc = lwip_send(socketFd, deferred_ + deferredHead_, contiguous,
                               MSG_DONTWAIT);
      if (rc > 0) {
        deferredHead_ = (deferredHead_ + (size_t)rc) % BLYNK_STOCK_DEFERRED_TX_BYTES;
        deferredCount_ -= (size_t)rc;
        if (deferredCount_ == 0) {
          ++deferredDrains_;
          Serial.printf("[Blynk DEFERRED TX] accepted-by-lwIP age=%lums drains=%lu; awaiting matching Blynk RESPONSE\n",
                        (unsigned long)(millis() - deferredSince_),
                        (unsigned long)deferredDrains_);
          deferredHead_ = 0;
          deferredSince_ = 0;
        }
      } else if (rc < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        blynkTransportLastErrno = errno;
        ++blynkTransportSocketErrors;
        WiFiClient::stop();
        clearDeferred();
      }
    }

  private:
    uint8_t deferred_[BLYNK_STOCK_DEFERRED_TX_BYTES];
    size_t deferredHead_;
    size_t deferredCount_;
    uint32_t deferredSince_;
    uint32_t deferredEvents_;
    uint32_t deferredDrains_;
    uint32_t recoverySince_;
    bool awaitingPeerConfirmation_;
    bool peerProbeArmed_;
    uint16_t peerProbeMsgId_;

    void clearDeferred() {
      deferredHead_ = 0;
      deferredCount_ = 0;
      deferredSince_ = 0;
      awaitingPeerConfirmation_ = false;
      peerProbeArmed_ = false;
      peerProbeMsgId_ = 0;
      recoverySince_ = 0;
    }

    bool appendDeferred(const uint8_t *data, size_t length) {
      if (length > BLYNK_STOCK_DEFERRED_TX_BYTES - deferredCount_) return false;
      const size_t tail = (deferredHead_ + deferredCount_) % BLYNK_STOCK_DEFERRED_TX_BYTES;
      const size_t first = min(length,
          (size_t)BLYNK_STOCK_DEFERRED_TX_BYTES - tail);
      memcpy(deferred_ + tail, data, first);
      if (length > first) memcpy(deferred_, data + first, length - first);
      deferredCount_ += length;
      if (deferredSince_ == 0) {
        deferredSince_ = millis();
        ++deferredEvents_;
        awaitingPeerConfirmation_ = true;
        peerProbeArmed_ = false;
        recoverySince_ = deferredSince_;
        Serial.printf("[Blynk DEFERRED TX] backpressure queued=%u event=%lu; application TX gated until verified RESPONSE\n",
                      (unsigned)deferredCount_, (unsigned long)deferredEvents_);
      }
      return true;
    }
  };

  class BlynkWifiStockBounded : public BlynkProtocol<BlynkArduinoClient> {
    typedef BlynkProtocol<BlynkArduinoClient> Base;
  public:
    explicit BlynkWifiStockBounded(BlynkArduinoClient& transp) : Base(transp) {}

    void config(const char* auth, const char* domain = BLYNK_DEFAULT_DOMAIN,
                uint16_t port = BLYNK_DEFAULT_PORT) {
      Base::begin(auth);
      this->conn.begin(domain, port);
    }

    void config(const char* auth, IPAddress ip,
                uint16_t port = BLYNK_DEFAULT_PORT) {
      Base::begin(auth);
      this->conn.begin(ip, port);
    }
  };

  static BoundedWriteWiFiClient _blynkWifiClient;
  static BlynkArduinoClient _blynkTransport(_blynkWifiClient);
  BlynkWifiStockBounded Blynk(_blynkTransport);

  static void blynkProtocolTransportService() {
    _blynkWifiClient.serviceDeferred();
  }

  static bool blynkProtocolTransportPending() {
    return _blynkWifiClient.pendingBytes() != 0;
  }

  #include <esp_system.h>

  // v3.9.136 FIX: BLYNK_CONNECTED() conflicts with a weak default handler
  // already declared by the installed Blynk library (BlynkHandlers.h:228),
  // producing a C-linkage redeclaration error. Rather than depend on that
  // library-internal macro, detect the false->true transition of
  // Blynk.connected() directly from loop() and apply setNoDelay() there —
  // functionally identical (fires exactly once per successful connect,
  // right after it happens) without touching the BLYNK_CONNECTED() macro
  // at all.
  static bool blynkNoDelayConnectedPrev = false;

  static inline void blynkApplyNoDelayOnConnectEdge()
  {
    const bool nowConnected = Blynk.connected();
    if (nowConnected && !blynkNoDelayConnectedPrev) {
      // v3.9.174: use normal TCP batching.  Field testing disproved
      // TCP_NODELAY as a cure and this sketch emits many small virtualWrite
      // frames; coalescing reduces both lwIP queue pressure and 802.11 frames.
      // Heartbeats still leave on the normal Blynk cadence.
      _blynkWifiClient.setTimeout(BLYNK_TIMEOUT_MS);
      const int noDelayRc = _blynkWifiClient.setNoDelay(false);
      const int socketFd = _blynkWifiClient.fd();
      // v3.9.177: BlynkProtocol already probes every 10s and owns the 189s
      // deadline. A 15+3*5 second SO_KEEPALIVE policy killed the underlying TCP
      // socket after ~30s, before the captured 47-58s retransmission recovery.
      int keepalive = 0;
      int keepRc = -1;
      int keepaliveActual = -1;
      int keepActualRc = -1;
      if (socketFd >= 0) {
        keepRc = setsockopt(socketFd, SOL_SOCKET, SO_KEEPALIVE,
                            &keepalive, sizeof(keepalive));
        socklen_t keepaliveLen = sizeof(keepaliveActual);
        keepActualRc = getsockopt(socketFd, SOL_SOCKET, SO_KEEPALIVE,
                                  &keepaliveActual, &keepaliveLen);
      }
      Serial.printf("[Blynk TCP POLICY] fd=%d noDelay=%d/%d keepaliveDisableRc=%d actual=%d getRc=%d protocolPing=10s protocolDeadline=189s applicationAckBarrier=ON\n",
                    socketFd, _blynkWifiClient.getNoDelay() ? 1 : 0,
                    noDelayRc, keepRc, keepaliveActual, keepActualRc);
      DBG_PRINTF("[Blynk TRANSPORT] nonblocking deferredTx=%uB setTimeout=%lums noDelay=%d\n",
                 (unsigned)BLYNK_STOCK_DEFERRED_TX_BYTES,
                 (unsigned long)BLYNK_TIMEOUT_MS,
                 _blynkWifiClient.getNoDelay() ? 1 : 0);
    }
    blynkNoDelayConnectedPrev = nowConnected;
  }

  // v3.9.87: small diagnostic adapter for the STOCK WiFiClient transport.
  // This deliberately exposes only Arduino WiFiClient observations; it does
  // NOT access raw lwIP sockets and does NOT consume Blynk data.
  static uint32_t v61TestSequence = 0;
  static uint32_t forensicRspObsCount = 0;
  static uint32_t forensicRspNewCount = 0;
  static uint32_t forensicRspSamePeekCount = 0;
  static uint16_t forensicRspLastMsgId = 0;
  static uint8_t forensicRspLastCmd = 0;
  static uint16_t forensicRspLastField = 0;
  static uint32_t forensicRspLastSeenMillis = 0;
  static uint32_t forensicRspLastNewMillis = 0;
  static uint32_t forensicRspLastGapMs = 0;
  static uint32_t forensicRspMaxGapMs = 0;
  static uint32_t forensicRspLastV61Seq = 0;
  static uint32_t forensicRspLastV61SeqAtNew = 0;
  static uint32_t forensicRspLastNewAgeMs = 0;
  // v3.9.114: distinguish server PING frames (cmd=6) from RESPONSE frames
  // (cmd=0). A RESPONSE-header gap is not the same thing as Blynk's private
  // lastActivityIn timer: server PING traffic also counts as inbound activity.
  static uint32_t forensicPingObsCount = 0;
  static uint32_t forensicPingNewCount = 0;
  static uint32_t forensicPingLastMillis = 0;
  static uint32_t forensicPingLastGapMs = 0;
  static uint32_t forensicPingMaxGapMs = 0;
  static uint16_t forensicPingLastMsgId = 0;

  // v3.9.132: per-ping round-trip watch state. Armed by
  // blynkHeartbeatService() right before sending a PING with a rotating,
  // reserved msgId; cleared here in diagnosticResponseHeaderPeek() when a
  // header carrying that exact msgId is observed; checked for a stale
  // (never-answered) arm by blynkPingRttWatchCheckTimeout(), called
  // periodically from loop().
  static bool     blynkPingRttWatchArmed = false;
  static uint16_t blynkPingRttWatchMsgId = 0;
  static uint32_t blynkPingRttWatchStartMillis = 0;
  static uint32_t blynkPingRttAnsweredCount = 0;
  static uint32_t blynkPingRttUnansweredCount = 0;

  static void blynkPingRttWatchArm(uint16_t msgId, uint32_t startMillis)
  {
    if (blynkPingRttWatchArmed) {
      // Previous watch was never answered and never timed out before this
      // one armed — count it as unanswered rather than silently overwrite.
      ++blynkPingRttUnansweredCount;
      DBG_PRINTF("[Blynk PING RTT] msgId=%u was never answered before the next PING armed (unansweredCount=%lu)\n",
                 (unsigned)blynkPingRttWatchMsgId, (unsigned long)blynkPingRttUnansweredCount);
    }
    blynkPingRttWatchArmed = true;
    blynkPingRttWatchMsgId = msgId;
    blynkPingRttWatchStartMillis = startMillis;
  }

  // Call periodically (loop()) while a watch may be armed. If armed for
  // longer than the full heartbeat deadline with no match, the server
  // never responded to this specific PING within the window that would
  // otherwise cause a heartbeat-timeout disconnect — direct confirmation
  // rather than inference from general RX silence.
  static void blynkPingRttWatchCheckTimeout(uint32_t nowMs, uint32_t timeoutMs)
  {
    if (!blynkPingRttWatchArmed) return;
    if (nowMs - blynkPingRttWatchStartMillis < timeoutMs) return;
    ++blynkPingRttUnansweredCount;
    Serial.printf("[Blynk PING RTT] msgId=%u UNANSWERED after %lums — server never responded to this specific device-sent PING (answeredCount=%lu unansweredCount=%lu)\n",
                  (unsigned)blynkPingRttWatchMsgId, (unsigned long)(nowMs - blynkPingRttWatchStartMillis),
                  (unsigned long)blynkPingRttAnsweredCount, (unsigned long)blynkPingRttUnansweredCount);
    blynkPingRttWatchArmed = false;
  }

  class StockBlynkTransportDiagnostics {
  public:
    void diagnosticRawRxState() {
      const int queued = _blynkWifiClient.available();
      Serial.printf("[Blynk STOCK RX] connected=%d available=%d wifi=%d RSSI=%d heap=%u\n",
                    _blynkWifiClient.connected() ? 1 : 0,
                    queued,
                    (int)WiFi.status(),
                    WiFi.RSSI(),
                    ESP.getFreeHeap());
    }

    void diagnosticRxTrace(const char* tag) {
      const int queued = _blynkWifiClient.available();
      Serial.printf("[Blynk STOCK RX TRACE] %s connected=%d available=%d wifi=%d RSSI=%d\n",
                    tag ? tag : "TRACE",
                    _blynkWifiClient.connected() ? 1 : 0,
                    queued,
                    (int)WiFi.status(),
                    WiFi.RSSI());
    }

    // v3.9.110: passive five-byte Blynk header peek for the STOCK client.
    // Uses the socket fd with MSG_PEEK, so no bytes are consumed and Blynk.run()
    // remains the only code that actually parses the protocol stream.
    void diagnosticResponseHeaderPeek(const char* phase) {
      const int availableBefore = _blynkWifiClient.available();
      // Only attempt a header peek when the stock client reports at least a
      // complete 5-byte Blynk header. This keeps the forensic probe silent
      // during the normal empty-RX case.
      if (availableBefore < 5) return;
      int fd = -1;
      #if defined(ARDUINO_ARCH_ESP32)
        fd = _blynkWifiClient.fd();
      #endif
      uint8_t h[5] = {0,0,0,0,0};
      int n = -1;
      int e = 0;
      if (fd >= 0 && availableBefore >= 5) {
        errno = 0;
        n = lwip_recv(fd, h, sizeof(h), MSG_DONTWAIT | MSG_PEEK);
        e = (n < 0) ? errno : 0;
      }
      if (n == 5) {
        const uint8_t cmd = h[0];
        const uint16_t msgId = ((uint16_t)h[1] << 8) | h[2];
        const uint16_t field = ((uint16_t)h[3] << 8) | h[4];
        const uint16_t bodyLen = (cmd == 0) ? 0 : field;
        const uint32_t now = millis();

        // v3.9.132 DIAGNOSTIC: per-ping round-trip correlation. See
        // blynkPingRttWatchArm()/blynkHeartbeatService(). This checks EVERY
        // peeked inbound header (any cmd, not just RESPONSE) against the
        // currently-armed ping msgId, since we don't know in advance
        // whether the server answers PING with a plain RESPONSE (cmd=0) or
        // echoes something else.
        if (blynkPingRttWatchArmed && msgId == blynkPingRttWatchMsgId) {
          const uint32_t rttMs = now - blynkPingRttWatchStartMillis;
          ++blynkPingRttAnsweredCount;
          // v3.9.164: only the matching post-backpressure PING response may
          // reopen application publishing. An empty local socket queue alone
          // does not prove that the server received anything.
          if (cmd == BLYNK_CMD_RESPONSE) {
            _blynkWifiClient.confirmPeerProgress(msgId);
          }
          Serial.printf("[Blynk PING RTT] msgId=%u cmd=%u rttMs=%lu — server DID respond to this specific device-sent PING (answeredCount=%lu)\n",
                        (unsigned)msgId, (unsigned)cmd, (unsigned long)rttMs, (unsigned long)blynkPingRttAnsweredCount);
          blynkPingRttWatchArmed = false;
        }

        // v3.9.173: normal heartbeats are generated inside BlynkProtocol, not
        // by the separate sketch RTT watcher. Feed every fresh RESPONSE to the
        // transport recovery latch; confirmPeerProgress() still requires the
        // deferred queue to be empty and enforces an ID when one was armed.
        if (cmd == BLYNK_CMD_RESPONSE) {
          _blynkWifiClient.confirmPeerProgress(msgId);
        }

        const bool sameHeader = (forensicRspObsCount > 0 &&
                                 cmd == forensicRspLastCmd &&
                                 msgId == forensicRspLastMsgId &&
                                 field == forensicRspLastField);
        // cmd=6 is BLYNK_CMD_PING. It is an inbound protocol activity event,
        // even though it is not a RESPONSE/PONG header. Track it separately.
        if (cmd == BLYNK_CMD_PING) {
          ++forensicPingObsCount;
          if (forensicPingLastMsgId != msgId) {
            ++forensicPingNewCount;
            if (forensicPingLastMillis != 0) {
              forensicPingLastGapMs = now - forensicPingLastMillis;
              if (forensicPingLastGapMs > forensicPingMaxGapMs) forensicPingMaxGapMs = forensicPingLastGapMs;
            }
            forensicPingLastMillis = now;
            forensicPingLastMsgId = msgId;
          }
          Serial.printf("[Blynk RX PING] phase=%s msgId=%u at=%lums newPing=%lu gap=%lums maxGap=%lums\n",
                        phase ? phase : "UNKNOWN", (unsigned)msgId, now,
                        (unsigned long)forensicPingNewCount,
                        (unsigned long)forensicPingLastGapMs,
                        (unsigned long)forensicPingMaxGapMs);
        }
        ++forensicRspObsCount;
        if (sameHeader) {
          ++forensicRspSamePeekCount;
        } else {
          ++forensicRspNewCount;
          if (forensicRspLastNewMillis != 0) {
            forensicRspLastGapMs = now - forensicRspLastNewMillis;
            if (forensicRspLastGapMs > forensicRspMaxGapMs) forensicRspMaxGapMs = forensicRspLastGapMs;
          }
          forensicRspLastNewMillis = now;
          forensicRspLastV61SeqAtNew = v61TestSequence;
        }
        forensicRspLastMsgId = msgId;
        forensicRspLastCmd = cmd;
        forensicRspLastField = field;
        forensicRspLastSeenMillis = now;
        forensicRspLastNewAgeMs = now - forensicRspLastNewMillis;
        Serial.printf("[Blynk RX RESPONSE HEADER] phase=%s avail=%d fd=%d peek=%d cmd=%u msgId=%u field=%u bodyLen=%u hex=%02X %02X %02X %02X %02X at=%lums same=%d obs=%lu new=%lu samePeek=%lu gap=%lums maxGap=%lums V61seq=%lu V61seqAtNew=%lu\n",
                      phase ? phase : "UNKNOWN", availableBefore, fd, n,
                      (unsigned)cmd, (unsigned)msgId, (unsigned)field, (unsigned)bodyLen,
                      h[0],h[1],h[2],h[3],h[4], now, sameHeader ? 1 : 0,
                      (unsigned long)forensicRspObsCount, (unsigned long)forensicRspNewCount,
                      (unsigned long)forensicRspSamePeekCount,
                      (unsigned long)forensicRspLastGapMs, (unsigned long)forensicRspMaxGapMs,
                      (unsigned long)v61TestSequence, (unsigned long)forensicRspLastV61SeqAtNew);
      } else {
        Serial.printf("[Blynk RX RESPONSE HEADER] phase=%s avail=%d fd=%d peek=%d errno=%d\n",
                      phase ? phase : "UNKNOWN", availableBefore, fd, n, e);
      }
    }

    void disconnect() {
      _blynkWifiClient.stop();
    }
  };

  static StockBlynkTransportDiagnostics _stockBlynkTransportDiagnostics;
#endif

#include <esp_mac.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_attr.h>              // RTC_DATA_ATTR
#include <esp_heap_caps.h>
#include <freertos/queue.h>
#include <TinyGPS++.h>
#include <Wire.h>
// ENS160: use ScioSense_ENS160 library (NOT Adafruit_ENS160 which does not exist)
// Install: https://github.com/sciosense/ENS160_driver  or via Library Manager
#include "ScioSense_ENS160.h"
#include <Adafruit_AHTX0.h>      // AHT20/AHT21 temp+hum — install: Adafruit AHTX0
#include <Adafruit_SSD1306.h>
#include <driver/i2s_std.h>      // ESP-IDF 5.x standard I2S driver used by Arduino-ESP32 3.x
#include "esp_system.h"

// v3.9.52: this source is intentionally migrated for Arduino-ESP32 3.3.x.
// Refuse accidental builds with the old 2.x API set.
#ifndef ESP_ARDUINO_VERSION
  #error "Arduino-ESP32 core version macros are unavailable; use Arduino-ESP32 3.3.11"
#endif
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 0)
  #error "This firmware requires Arduino-ESP32 core 3.3.x (target: 3.3.11)"
#endif

// Core ownership is part of the design, not an optional tuning hint. These
// macros are supplied by Tools > Arduino Runs On / Events Run On. Refuse a
// build that would silently put Blynk or WiFi callbacks back on Core 1.
#if ARDUINO_RUNNING_CORE != 0
  #error "Select Tools > Arduino Runs On > Core 0"
#endif
#if ARDUINO_EVENT_RUNNING_CORE != 0
  #error "Select Tools > Events Run On > Core 0"
#endif
#if !defined(CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0) || !defined(CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0)
  #error "This build requires the ESP32 WiFi and lwIP tasks pinned to Core 0"
#endif

// ─── PERFORMANCE / I2C DIAGNOSTICS — declarations must precede PPS ISR ───
// Internal counters/timers run continuously; V34-V50 are published only
// by the 10-second diagnostic pathway.
volatile uint32_t perfPpsLastUs = 0;
volatile uint32_t perfMaxLoopGapMs = 0;
volatile uint32_t perfMaxSensorCycleMs = 0;
volatile uint32_t perfMaxBlynkRunMs = 0;

// v3.9.54 diagnostics: record long Blynk.run() calls without changing the
// normal protocol timeout.  These are intentionally independent of DEBUGON
// so an actual long-run event remains visible in field logs.
volatile uint32_t blynkRunGuardCount = 0;
volatile uint32_t blynkRunLastGuardMs = 0;
volatile uint32_t blynkRunMaxGuardMs = 0;
volatile uint32_t perfMaxGpsFeedMs = 0;
volatile uint32_t perfMaxI2cPathMs = 0;
volatile uint32_t perfMaxFastSendMs = 0;
volatile uint32_t perfMaxSlowSendMs = 0;
volatile uint32_t perfMaxPpsLedResponseMs = 0;
volatile uint32_t perfLastI2cDevice = 0;
volatile uint32_t perfLastI2cOperation = 0;
volatile uint32_t perfI2cTimeoutCount = 0;
volatile uint32_t perfMaxEns160Ms = 0;
volatile uint32_t perfMaxAht21Ms = 0;
volatile uint32_t perfMaxBmi160Ms = 0;

enum : uint8_t { SENSOR_STATE_BOOT=0, SENSOR_STATE_RUN=1, SENSOR_STATE_I2C=2 };
enum : uint8_t { LOOP_STATE_BOOT=0, LOOP_STATE_RUN=1, LOOP_STATE_WIFI=2,
                 LOOP_STATE_BLYNK=3, LOOP_STATE_HEALTH=4 };
volatile uint8_t sensorTaskState = SENSOR_STATE_BOOT;
volatile uint8_t loopState = LOOP_STATE_BOOT;

static inline void perfRecordMax(volatile uint32_t& dst, uint32_t value)
{
  if (value > dst) dst = value;
}

static inline uint32_t perfPpsAgeMs()
{
  uint32_t lastUs = perfPpsLastUs;
  if (lastUs == 0) return 0;
  return (uint32_t)((micros() - lastUs) / 1000UL);
}

static inline void perfI2cBegin(uint32_t device, uint32_t operation)
{
  perfLastI2cDevice = device;
  perfLastI2cOperation = operation;
  sensorTaskState = SENSOR_STATE_I2C;
}

static inline void perfI2cTimeout()
{
  perfI2cTimeoutCount++;
}

// v2.1 previously used the legacy <driver/i2s.h> here. That header put the
// firmware at risk of the "ADC: CONFLICT driver_ng" abort because the legacy
// I2S driver and the driver_ng ADC API used by analogRead() cannot safely
// coexist. <driver/i2s_std.h> is the new-generation I2S driver family and
// does not touch the ADC driver at all — see v2.2 CHANGES note above.
// NOTE: HISTORICAL ONLY — the following legacy-driver workaround is retained
// as a changelog note. v3.9.52 now uses driver/i2s_std.h directly on core 3.x:
//   1. Call analogReadResolution() + analogSetPinAttenuation() per pin in setup() FIRST.
//   2. Do NOT call analogSetAttenuation() (global) — it uses driver_ng internally.
//   3. Call i2s_driver_install() (initINMP441) LAST in setup(), after all ADC config.
// ^ Superseded by v2.2: the above three-step workaround is no longer required
//   now that INMP441 uses <driver/i2s_std.h>, which never touches the ADC.
//   Kept here for historical context only — the current initINMP441() below
//   uses the new driver and can be initialised in ANY order relative to ADC.

// ─── FEATURE SWITCHES ────────────────────────────────────────────────────────
#define DISPLAYON      false
#define WIFI           true
// v3.9.19: user-selectable ESP32 WiFi modem-sleep.  Set false to keep
// the radio fully awake; set true to restore normal WiFi power saving.
#define WIFI_SLEEP     false
// v3.9.143 TEST SWITCH: 1 = restrict the ESP32 STA to 802.11b/g (no HT/A-MPDU
// aggregation, like the ESP8266). If the ESP32->AP black-holes (minutes of lost
// frames while still associated) disappear with this set to 1, the AP/driver
// is mishandling 11n aggregation. The A/B test did not prevent disconnects,
// so normal 802.11b/g/n operation is restored.
#define WIFI_TEST_LEGACY_BG 0
#define COsensorThere   false  // MQ-7 not installed; never infer presence from a floating ADC
#define DUSTsensorThere false  // GP2Y1010 not installed; keep V18 unavailable
// [SUPERSEDED v2.4] IMUsensorThere (JY-901 UART) removed. BMI160sensorThere
// below now gates the ONLY IMU in the system — both inertial nav AND the
// Roll/Pitch/Yaw/temp feeding V8-V11.
#define ENSsensorThere true    // ENS160 + AHT2x combo board
#define INMPsensorThere true   // INMP441 I2S microphone
#define BMI160sensorThere true // GY-BMI160 6-DOF IMU for inertial dead-reckoning (v2.3)
#define PPSsensorThere    true // GPS 1PPS lock-status interrupt (v2.3)

bool ten_mins_autoreset = false;

// ─── INERTIAL NAVIGATION — user configurable (v2.3) ─────────────────────────
float inav_update_hz          = 10.0f; // interpolation output frequency (Hz)
int   inav_stale_after_misses = 3;     // consecutive missed GPS fixes before "stale" state
int   inav_max_loss_minutes   = 2;     // stop interpolating after this many minutes without GPS
                                        // (shortened from 5 — bounds worst-case gyro dead-reckoning
                                        // drift distance; see V33 map-source note below)

#define SENSOR_UNAVAILABLE (-999.0f)

volatile bool setupDone = false;

// ─── VOLTAGE DIVIDER  10kΩ + 10kΩ → Vout@5V = 2.5V ─────────────────────────
const float R_DIVIDER_SERIES = 10.0f;
const float R_DIVIDER_GND    = 10.0f;
float divider_scale = 2.0f;
float divider_ratio = 0.5f;

// ─── OLED ────────────────────────────────────────────────────────────────────
#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// ─── GPS — ATGM336H on UART1 GPIO13(RX)/GPIO23(TX) ──────────────────────────
// ATGM336H outputs standard NMEA0183 at various baud rates (9600/38400/57600/115200).
// Auto-baud detection at startup, then configured to 115200 baud + 10Hz update rate.
// PPS signal (V26) must be detected before baud detection begins (30s timeout).
// GPIO13 chosen to free GPIO22 for I2C SCL (ENS160+AHT2x).
#define GPS_RX_PIN    13
#define GPS_TX_PIN    23
#define GPS_BAUD      115200  // Target baud rate (auto-detection then PMTK config)
HardwareSerial gps_serial(1);
TinyGPSPlus    gps;

// v3.9.32 GPS UART diagnostics. These counters distinguish a GPS UART
// transport problem from a TinyGPS++ parsing/fix problem. They do not alter
// GPS processing or the PPS input on GPIO5.
volatile uint32_t gpsUartBytesReceived = 0;
volatile uint32_t gpsUartBytesThisBoot = 0;
volatile uint32_t gpsLastByteMillis = 0;
// v3.9.32 GPS parser diagnostics: distinguish valid NMEA from arbitrary UART bytes.
volatile uint32_t gpsDollarCount = 0;
volatile uint32_t gpsCrlfCount = 0;

// v3.9.32 GPS RAW-NMEA CAPTURE (diagnostic only).
// Keep a small number of complete NMEA-looking lines in RAM, then print them
// periodically. This lets us inspect the exact bytes reaching GPIO13 without
// flooding Serial while the UART is being serviced. No GPS processing is
// changed: every byte is still passed to TinyGPS++ exactly as before.
#define GPS_RAW_CAPTURE_LINES 6
#define GPS_RAW_LINE_MAX      192
static char gpsRawLines[GPS_RAW_CAPTURE_LINES][GPS_RAW_LINE_MAX];
static uint16_t gpsRawLineLengths[GPS_RAW_CAPTURE_LINES] = {0};
static uint8_t gpsRawLineCount = 0;
static uint16_t gpsRawLineWorkLen = 0;
static bool gpsRawLineActive = false;
static bool gpsRawLineOverflow = false;

static void gpsRawCaptureReset()
{
  gpsRawLineCount = 0;
  gpsRawLineWorkLen = 0;
  gpsRawLineActive = false;
  gpsRawLineOverflow = false;
  for (uint8_t i = 0; i < GPS_RAW_CAPTURE_LINES; ++i) {
    gpsRawLineLengths[i] = 0;
    gpsRawLines[i][0] = '\0';
  }
}

// Separate fixed work buffer avoids any dynamic String allocation and makes
// the capture independent of how many lines have already been completed.
static char gpsRawWorkLine[GPS_RAW_LINE_MAX];

static void gpsRawCaptureByteSafe(uint8_t c)
{
  if (c == '$') {
    gpsRawLineActive = true;
    gpsRawLineWorkLen = 0;
    gpsRawLineOverflow = false;
  }
  if (!gpsRawLineActive) return;

  if (gpsRawLineWorkLen < GPS_RAW_LINE_MAX - 1) {
    gpsRawWorkLine[gpsRawLineWorkLen++] = (char)c;
    gpsRawWorkLine[gpsRawLineWorkLen] = '\0';
  } else {
    gpsRawLineOverflow = true;
  }

  if (c == '\n') {
    if (gpsRawLineCount < GPS_RAW_CAPTURE_LINES) {
      const uint8_t slot = gpsRawLineCount++;
      const uint16_t n = gpsRawLineWorkLen < GPS_RAW_LINE_MAX - 1
                       ? gpsRawLineWorkLen : GPS_RAW_LINE_MAX - 1;
      memcpy(gpsRawLines[slot], gpsRawWorkLine, n);
      gpsRawLines[slot][n] = '\0';
      gpsRawLineLengths[slot] = n;
    }
    gpsRawLineActive = false;
    gpsRawLineWorkLen = 0;
    gpsRawLineOverflow = false;
  }
}

static int gpsHexNibble(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool gpsRawChecksum(const char* line, uint16_t len, uint8_t& calculated, int& transmitted)
{
  calculated = 0;
  transmitted = -1;
  if (!line || len < 5 || line[0] != '$') return false;

  int star = -1;
  for (uint16_t i = 1; i < len; ++i) {
    if (line[i] == '*') { star = (int)i; break; }
    if (line[i] == '\r' || line[i] == '\n') break;
    calculated ^= (uint8_t)line[i];
  }
  if (star < 0 || star + 2 >= len) return false;
  const int hi = gpsHexNibble(line[star + 1]);
  const int lo = gpsHexNibble(line[star + 2]);
  if (hi < 0 || lo < 0) return false;
  transmitted = (hi << 4) | lo;
  return true;
}

static void gpsPrintRawCapture()
{
  if (GPS_DEBUG_PRINTOUT == true) {
    if (gpsRawLineCount == 0) {
      Serial.println("[GPS RAW] no complete $...\n sentence captured yet");
      return;
    }

    Serial.printf("[GPS RAW] captured %u complete NMEA-looking line(s):\n",
                  (unsigned)gpsRawLineCount);

    for (uint8_t i = 0; i < gpsRawLineCount; ++i) {
      const char* line = gpsRawLines[i];
      const uint16_t len = gpsRawLineLengths[i];
      uint8_t calc = 0;
      int tx = -1;
      const bool haveChecksum = gpsRawChecksum(line, len, calc, tx);

      Serial.printf("[GPS RAW %u] ", (unsigned)(i + 1));
      for (uint16_t j = 0; j < len; ++j) {
        const uint8_t c = (uint8_t)line[j];
        if (c == '\r' || c == '\n') continue;
        if (c >= 32 && c <= 126) {
          Serial.write(c);
        } else {
          Serial.printf("\\x%02X", (unsigned)c);
        }
      }
      if (haveChecksum) {
        Serial.printf(" | checksum RX=%02X CALC=%02X %s\n",
                      (unsigned)tx, (unsigned)calc,
                      tx == calc ? "VALID" : "BAD");
      } else {
        Serial.println(" | checksum: NOT DECODABLE");
      }
    }

    gpsRawCaptureReset();
  }
}

// v3.5: Capture TinyGPS++ location-update state before any normal GPS
// latitude/longitude access below. TinyGPS++ clears location.isUpdated()
// when lat()/lng() are read, so the raw-GPS reporting path must not consume
// the update flag before inertialNav() gets a chance to use the same fix.
bool gpsLocationUpdatedThisCycle = false;

// [SUPERSEDED v2.4] JY-901 UART2 (GPIO16/17) removed entirely — BMI160 (I2C,
// declared below near the ENS160/AHT2x section) is now the only IMU.
// GPIO16/GPIO17 are free.

// ─── ENS160 + AHT2x — I2C GPIO21(SDA)/GPIO22(SCL) ──────────────────────────
// ScioSense_ENS160 API:
//   ens160.begin()           — init, returns void
//   ens160.available()       — returns bool (true if responding)
//   ens160.setMode(ENS160_OPMODE_STD) — set standard operating mode
//   ens160.set_envdata(tempC_int, hum_int) — compensation (int values)
//   ens160.measure(true)     — trigger measurement
//   ens160.measureRaw(true)  — trigger raw measurement
//   ens160.getAQI()          — uint8_t  1–5 index
//   ens160.getTVOC()         — uint16_t ppb
//   ens160.geteCO2()         — uint16_t ppm equivalent CO2
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);  // 0x53 (ADD pin → VCC on this board)
Adafruit_AHTX0   aht;

// ─── INMP441 I2S microphone ──────────────────────────────────────────────────
#define I2S_PORT        I2S_NUM_0
#define I2S_SCK_PIN     25   // BCLK
#define I2S_WS_PIN      26   // LRCLK  [ADC2 output only — no WiFi ADC conflict]
#define I2S_SD_PIN      33   // SD data in [freed from NO2 analogue]
#define I2S_SAMPLE_RATE 16000
#define I2S_BUF_SAMPLES 256

// Arduino-ESP32 3.x / ESP-IDF 5.x standard I2S RX channel.
static i2s_chan_handle_t inmp441RxHandle = NULL;
// Arduino-ESP32 3.3.x: microphone uses the standard I2S_NUM_0 driver.
bool inmp441Initialized = false;

// ─── ANALOG PINS (ADC1 only) ─────────────────────────────────────────────────
// All 5V sensor outputs require 10kΩ + 10kΩ voltage divider before ESP32 ADC.
// ML8511 UV is 3.3V ratiometric — direct connection, no divider.
#define CO_ADC_PIN    36    // VP ADC1_CH0 input-only — MQ-7 A0 (CO sense, via divider)
#define CO_A1_PIN     39    // VN ADC1_CH3 input-only — MQ-7 A1 (heater feedback, via divider)
#define UV_OUT_PIN    34    //    ADC1_CH6 input-only — ML8511 OUT (direct 3.3V)
#define UV_REF_PIN    35    //    ADC1_CH7 input-only — ML8511 REF (direct 3.3V)
#define DUST_ADC_PIN  32    //    ADC1_CH4 — GP2Y1010 Vo (via 10k+10k divider)

// ─── DIGITAL PINS ────────────────────────────────────────────────────────────
#define ledPin11   14    // GREEN  CO ≤ 10 ppm
#define ledPin12   27    // ORANGE CO 10–20 ppm  (ADC2, output only — no WiFi conflict)
#define ledPin13   15    // RED    CO > 20 ppm   (GPIO26 freed for I2S WS)
#define buzzPin    19    // Passive buzzer
#define dustLED    18    // Sharp GP2Y1010 IR LED (active LOW)
#define CO_PWM_PIN  4    // CO heater D2 PWM (direct to NPN base via 1kΩ on board)
#define PPS_PIN     5    // GPS 1PPS output → ESP32 GPIO5 (free since DHT21 removed in v2.0)
                          // 3.3V push-pull, direct wire, no divider needed
#define PPS_LED_PIN 2     // ESP-32 board D2 LED — flashes for 250ms on each GPS PPS
// Some ESP32 boards wire the onboard D2/GPIO2 LED active-LOW. The current
// station board uses that polarity, so LOW means LED ON and HIGH means OFF.
// Change to true only if your particular board has an active-HIGH D2 LED.
#define PPS_LED_ACTIVE_HIGH true

// ─── PPS ISR state (v2.3) ────────────────────────────────────────────────────
// Single-word volatiles are atomic on Xtensa — no critical section needed for
// the simple diagnostic reads in ppsIsLocked().
volatile unsigned long ppsLastMicros  = 0;
volatile unsigned long ppsIntervalUs  = 0;
volatile uint32_t      ppsPulseCount  = 0;
// PPS LED requests are handled by sensorTask, not directly by the ISR.
// This keeps the ISR limited to timestamp/counter/event work and also means
// the LED is a useful indication that the normal sensor application is alive.
volatile uint32_t      ppsLedPending = 0;
portMUX_TYPE ppsLedMux = portMUX_INITIALIZER_UNLOCKED;

// PPS LED timing is owned by sensorTask (Core 0).  No cross-core LED state is
// modified by the ISR beyond the atomic pending-event counter above.
// The LED uses an absolute OFF deadline so each PPS creates one fixed 250ms
// pulse and a later PPS cannot accidentally extend an already active pulse.
unsigned long ppsLedOffMillis = 0;
bool ppsLedIsOn = false;

// v3.6: PPS LED service runs in its own small FreeRTOS task.
// This keeps the 250ms LED pulse independent of the sensorTask timing.
// A slow I2C/sensor cycle can therefore never turn a 250ms PPS pulse into
// a continuously illuminated LED.
TaskHandle_t ppsLedTaskHandle = NULL;

// Keep the LED polarity in one place. GPIO2/D2 is a board LED output, and
// different ESP32 boards wire that LED with different polarities.
// Using a helper prevents the PPS state machine from accidentally assuming
// HIGH=ON when the physical LED is active-LOW.
void setPpsLed(bool on)
{
  digitalWrite(PPS_LED_PIN, PPS_LED_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                                 : (on ? LOW : HIGH));
}

// Forward declarations for performance diagnostics used by the PPS path.
static inline void perfRecordMax(volatile uint32_t& dst, uint32_t value);
static inline uint32_t perfPpsAgeMs();

void IRAM_ATTR ppsISR()
{
  unsigned long now = micros();
  if (ppsLastMicros != 0) ppsIntervalUs = now - ppsLastMicros;
  ppsLastMicros = now;
  ppsPulseCount++;
  // Performance diagnostic: timestamp the actual PPS ISR event.
  perfPpsLastUs = now;

  // Do NOT call digitalWrite() here. The ISR only records the PPS event and
  // wakes the dedicated LED task. The LED operation itself remains in normal
  // application context.
  portENTER_CRITICAL_ISR(&ppsLedMux);
  ppsLedPending++;
  portEXIT_CRITICAL_ISR(&ppsLedMux);

  if (ppsLedTaskHandle != NULL) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ppsLedTaskHandle, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

// v3.6: Dedicated PPS LED task. Every electrical PPS event received on GPIO5
// produces one fixed 250ms LED pulse. The LED is OFF at all other times.
// This is intentionally independent of sensorTask so slow sensor/I2C work
// cannot stretch the LED pulse or make it appear continuously ON.
void ppsLedTask(void *parameter)
{
  (void)parameter;

  for (;;) {
    // Sleep until the GPIO5 PPS ISR reports an event.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t pendingPps;
    portENTER_CRITICAL(&ppsLedMux);
    pendingPps = ppsLedPending;
    ppsLedPending = 0;
    portEXIT_CRITICAL(&ppsLedMux);

    if (pendingPps == 0) continue;

    // Performance diagnostic: measure scheduling latency from the PPS ISR
    // to the dedicated LED task. This does not affect the LED state machine.
    uint32_t perfPpsLedResponseStartUs = micros();
    uint32_t perfPpsEventUs = perfPpsLastUs;
    if (perfPpsEventUs != 0)
      perfRecordMax(perfMaxPpsLedResponseMs,
                    (uint32_t)((perfPpsLedResponseStartUs - perfPpsEventUs + 999UL) / 1000UL));

    // One PPS event = one 250ms ON pulse.
    setPpsLed(true);
    ppsLedIsOn = true;
    ppsLedOffMillis = millis() + 250UL;

    vTaskDelay(pdMS_TO_TICKS(250));

    // Always switch OFF after the fixed pulse, regardless of PPS lock state.
    setPpsLed(false);
    ppsLedIsOn = false;

    // If multiple PPS events accumulated while the 250ms pulse was active,
    // service them one-by-one. Normal GPS PPS is one event per second, so
    // under normal operation this path is not normally used.
    while (pendingPps > 1) {
      pendingPps--;

      setPpsLed(true);
      ppsLedIsOn = true;
      ppsLedOffMillis = millis() + 250UL;

      vTaskDelay(pdMS_TO_TICKS(250));

      setPpsLed(false);
      ppsLedIsOn = false;
    }
  }
}

// Locked = a pulse arrived within the last 2s AND its interval was within
// ±10% of the expected 1.000s. Tolerant enough for jitter, tight enough to
// reject a GPS that is powered but not yet timing-locked.
bool ppsIsLocked()
{
  if (ppsLastMicros == 0) return false;
  unsigned long sinceLast = micros() - ppsLastMicros;  // unsigned sub handles wrap
  if (sinceLast > 2000000UL) return false;
  unsigned long iv = ppsIntervalUs;
  if (iv < 900000UL || iv > 1100000UL) return false;
  return true;
}

// ─── LEDC ────────────────────────────────────────────────────────────────────
#define LEDC_FREQ_CO 5000
#define LEDC_RES_CO  8
// v3.9.4: keep CO and buzzer in different LEDC channel groups. ESP32 channels
// 0-7 are group 0 and 8-15 are group 1, so the two peripherals cannot share
// the same hardware timer when the buzzer frequency changes.
#define LEDC_CHANNEL_CO     0
#define LEDC_CHANNEL_BUZZER 8

// ─── GY-BMI160 6-DOF IMU — I2C, shares bus with ENS160/AHT2x (v2.3) ─────────
// Minimal register-level driver — no external library, avoids adding another
// dependency that could collide with the ADC/I2S driver families (see v2.1/
// v2.2 notes above for why that risk is taken seriously in this project).
#define BMI160_ADDR         0x69   // 0x69 --- SDO/SA0 pulled HIGH on the GY-BMI160 board
uint8_t bmi160ActiveAddr = BMI160_ADDR;  // v3.9.32: runtime address; auto-detect 0x69/0x68
#define BMI160_REG_CHIPID   0x00
#define BMI160_REG_DATA     0x0C   // GYR_X_L .. ACC_Z_H, 12 bytes contiguous
#define BMI160_REG_TEMP     0x20   // die temperature, 2 bytes (v2.4, feeds V11)
#define BMI160_REG_ACC_CONF  0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF  0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_CMD       0x7E
#define BMI160_CMD_SOFTRESET   0xB6
#define BMI160_CMD_ACC_NORMAL  0x11
#define BMI160_CMD_GYR_NORMAL  0x15
#define BMI160_CHIPID_EXPECTED 0xD1
#define BMI160_ACC_RANGE_G     4.0f    // ±4g   (ACC_RANGE=0x05)
#define BMI160_GYR_RANGE_DPS   500.0f  // ±500dps (GYR_RANGE=0x02)

// ─── INERTIAL NAV — EKF tuning constants (v2.3) ─────────────────────────────
// State x = [E(m), N(m), v(m/s), heading(rad, 0=North, clockwise+)]
#define EARTH_RADIUS_M         6371000.0f
#define INAV_R_POS             25.0f     // GPS position meas. variance (m^2), ~5m std
#define INAV_R_SPEED           0.25f     // GPS speed meas. variance (m/s)^2, ~0.5 m/s std
#define INAV_R_HEADING         0.00762f  // GPS course meas. variance (rad^2), ~5deg std
#define INAV_Q_POS             0.02f     // process noise, position (m^2/predict step)
#define INAV_Q_SPEED           0.05f     // process noise, speed ((m/s)^2/predict step)
#define INAV_Q_HEADING         0.000306f // process noise, heading (rad^2/step), ~1deg std
#define INAV_ZUPT_ACCEL_TOL_G  0.03f     // |accel_mag - 1g| under this = "not accelerating"
#define INAV_ZUPT_GYRO_TOL_DPS 2.0f      // |gyro| under this = "not rotating"
#define INAV_ZUPT_DURATION_MS  500       // sustained stillness before ZUPT fires
#define INAV_ZUPT_R_V          0.01f     // tight variance on the zero-velocity pseudo-measurement
#define INAV_MIN_COURSE_SPEED_MPS 0.5f   // below this, GPS course is unreliable — skip heading update

// IMU variance-based stillness detector. ZUPT above reacts to a single
// instantaneous sample being close to "at rest" — but a cheap GPS module
// commonly reports 0.3-1.0 m/s of speed noise while genuinely stationary
// (multipath/doppler jitter), which sits ABOVE INAV_MIN_COURSE_SPEED_MPS and
// gets treated as real motion by the full GPS update. Tracking the rolling
// VARIANCE of accel/gyro magnitude (not just their instantaneous value)
// distinguishes "actually moving slowly" from "stationary, but the sensor/GPS
// reading is noisy" — a single sample can be noisy in either direction, but
// noise doesn't sustain low variance across a window the way true stillness
// does.
#define INAV_IMU_VAR_ALPHA         0.15f   // EMA smoothing factor for the variance tracker
#define INAV_IMU_ACCEL_VAR_TOL_G2  0.0016f // accel-magnitude variance tolerance (g^2), ~0.04g std
#define INAV_IMU_GYRO_VAR_TOL_DPS2 2.25f   // gyro-magnitude variance tolerance (dps^2), ~1.5dps std
#define INAV_IMU_STILL_DURATION_MS 800     // sustained low-variance time before trusting "confirmed stationary"
#define INAV_STATE_GPS_FIX     0
#define INAV_STATE_IMU_RECENT  1
#define INAV_STATE_IMU_STALE   2
#define INAV_STATE_LOST        3
#define INAV_STATE_NO_FIX_YET  4

// ─── TIMEOUTS & INTERVALS ────────────────────────────────────────────────────
#define GPS_TIMEOUT_MS        15000
#define IMU_TIMEOUT_MS         5000
#define ENS_MAX_FAILS             5
#define ENS_RETRY_MS          10000
#define ENS_MEASURE_TIMEOUT_MS 1500UL
#define CO_PHASE_MAX_MS      180000
#define WIFI_RECONNECT_MS     30000
#define ENG_MSG_INTERVAL      15000
#define BLYNK_SEND_FAST_MS     20000
// v3.9.17: Do not rely only on the legacy 60-second Blynk heartbeat.
// The legacy library can leave a long window in which a stale/half-open TCP
// session is not actively tested by the application. Send an explicit Blynk
// protocol PING every 20s while connected. PING is a protocol command (not a
// virtualWrite), so it is not subject to BLYNK_MSG_LIMIT. The normal Blynk
// protocol still owns connection state and will update lastActivityIn when the
// server replies.
#define BLYNK_FORCE_PING_MS    10000UL
#define BLYNK_SEND_SLOW_MS    60000
// v3.9.43: blynkSendDiagnostics() (V34-V60, 22 fields — retained maxima and
// counters, not live sensor data) previously ran on the same 10s cadence as
// time-sensitive SLOW telemetry. It is the single largest per-cycle
// contributor to Blynk server write load of any TX group. Slowed to 30s to
// reduce steady-state server-side load (a candidate factor in the write-
// stall/EAGAIN issue investigated in the v3.9.43 recovery-ladder fix above —
// less write volume gives the server's socket-read loop more headroom even
// if it does not fully eliminate an occasional stall). FAST/SLOW/V4 are
// unchanged and still carry all real-time sensor and position data.
#define BLYNK_SEND_DIAG_MS    120000UL
#define MAP_SEND_INTERVAL_MS  120000   // V33 Map widget point rate — deliberately slower
                                      // than FAST/SLOW telemetry to avoid flooding the
                                      // Blynk Map widget with a marker every few seconds
#define SENSOR_TASK_PERIOD_MS     25
#define GPS_FEED_MS               150
#define IMU_POLL_MS               50
#define BUZZER_GPS_INTERVAL     5000
#define VWRITE_GAP_MS            10

// ─── PERFORMANCE DIAGNOSTIC VIRTUAL PINS ───────────────────────────────────
// Internal timing/counters run continuously; these pins are published
// only once every BLYNK_SEND_DIAG_MS (120s, see v3.9.43) by blynkSendDiagnostics().
#define V35 35  // PPS event count
#define V36 36  // milliseconds since last PPS
#define V37 37  // maximum Arduino loop gap (ms)
#define V38 38  // maximum sensor-task cycle duration (ms)
#define V39 39  // maximum Blynk.run() duration (ms)
#define V40 40  // maximum GPS feed duration (ms)
#define V41 41  // maximum I2C sensor-path duration (ms)
#define V42 42  // maximum FAST telemetry duration (ms)
#define V43 43  // maximum SLOW telemetry duration (ms)
#define V44 44  // maximum PPS ISR-to-LED-task response (ms)
#define V45 45  // last I2C device ID (0=none, 1=ENS160, 2=AHT21, 3=BMI160)
#define V46 46  // last I2C operation ID (0=none, 1=read, 2=measurement/command, 3=configuration)
#define V47 47  // cumulative I2C transaction error count
#define V48 48  // maximum AHT21 read duration (ms)
#define V49 49  // maximum ENS160 operation/read duration (ms)
#define V50 50  // maximum BMI160 raw-read duration (ms)
#define V51 51  // previous-run RTC crash checkpoint summary (string)
#define V52 52  // last Core-1 execution stage (string)
#define V53 53  // maximum Core-1 stage duration (ms)
#define V54 54  // Core-1 stall event count
#define V56 56  // last Blynk TX V-pin
#define V57 57  // last Blynk virtualWrite() duration (ms)
#define V58 58  // maximum Blynk virtualWrite() duration (ms)
#define V59 59  // cumulative Blynk virtualWrite() call count
#define V60 60  // last Blynk TX value length
#define V61 61  // unified 1-second sensor snapshot part A (CSV string)
#define V62 62  // unified 1-second sensor snapshot part B
#define V63 63  // current queued TX item count
#define V64 64  // last transmitted V-pin
#define V65 65  // maximum virtualWrite duration, ms (CSV string)

// ─── CO CLOSED-LOOP HEATER TARGETS ───────────────────────────────────────────
// A1 feedback divider scales 5V→2.5V at ADC. We target the ADC-side voltage.
// Physical heater voltages: heat=5.0V, measure=1.4V
// At ADC after 10k+10k divider: heat=2.5V, measure=0.7V
#define CO_HEAT_TARGET_V   2.50f   // ADC target during heating phase (=5V at sensor)
#define CO_MEAS_TARGET_V   0.70f   // ADC target during measurement phase (=1.4V at sensor)
#define CO_CLOSED_LOOP_TOL 0.05f   // ±tolerance in V at ADC before adjusting duty

// ─── STATUS FLAGS ────────────────────────────────────────────────────────────
#define STATUS_WIFI_OK   (1<<0)
#define STATUS_GPS_OK    (1<<1)
#define STATUS_IMU_OK    (1<<2)
#define STATUS_ENS_OK    (1<<3)    // was DHT_OK, now ENS160+AHT2x
#define STATUS_CO_FAULT  (1<<4)
#define STATUS_GPS_FAULT (1<<5)
#define STATUS_IMU_STUCK (1<<6)
#define STATUS_ENS_FAULT (1<<7)    // was DHT_FAULT
#define STATUS_CO_NO_SNS (1<<8)

// ─── CO CALIBRATION ──────────────────────────────────────────────────────────
// sensor_reading_clean_air: raw 12-bit ADC at GPIO36 (after divider) in fresh air
float reference_resistor_kOhm   = 9.98f;
float sensor_reading_clean_air  = 600.65f;
float sensor_reading_100_ppm_CO = -1.0f;

// ─── SHARED DATA STRUCT ──────────────────────────────────────────────────────
struct SensorData {
  float    co_ppm, co_raw, co_heater_v; // heater_v = actual A1 voltage (×divider_scale)
  byte     co_phase;
  bool     co_fault, co_no_sensor;
  float    tvoc_ppb, eco2_ppm, aqi;     // ENS160
  float    uvi;
  float    sound_db;                    // INMP441 dB SPL estimate
  float    dust_mg;
  float    temp, hum;                   // AHT2x
  bool     ens_fault;
  float    angle[3], imu_temp;
  bool     imu_ok;
  double   lat, lng;
  float    hdop, sats;
  bool     gps_fix;
  long     gps_unix_time;     // v2.5: UNIX epoch seconds from GPS UTC date/time, used as
                              // the Blynk Map (V33) point index — 0 if GPS date/time invalid
  int32_t  rssi;
  uint8_t  wifi_qual;
  uint32_t status_flags;
  char     eng_msg[128];
  uint32_t eng_msg_at_ms;                         // Core-local message timestamp
  int      rand_num;
  // v2.3 additions — additive only, nothing above this line changed
  bool     pps_locked;
  int      inav_state;                           // see INAV_STATE_* defines
  double   inav_lat, inav_lng;                   // EKF-fused position
  float    inav_alt;                             // held from last valid GPS fix (no baro on BMI160)
  float    inav_speed_mps, inav_heading_deg;     // EKF-fused speed/heading
};

SensorData        sd;
SemaphoreHandle_t dataMutex;
QueueHandle_t     sensorSnapshotQueue = NULL;

// V61 and V62 are two protocol messages but one database record. Their final
// CSV strings are sealed together on Core 1 so the record key cannot change
// between formatting the two halves. 256 matches BLYNK_TX_VALUE_LEN below;
// keeping the literal here avoids making the shared-data section depend on the
// later TX scheduler declarations.
struct Core1TelemetryPair {
  uint32_t sequence;
  uint32_t deviceMs;
  long gpsUnix;
  char v61[256];
  char v62[256];
};

struct Core0ConnectionSnapshot {
  int32_t rssi;
  uint8_t wifiQuality;
  bool wifiConnected;
};

QueueHandle_t telemetryPairQueue = NULL;       // Core 1 producer -> Core 0 network owner

static volatile uint32_t telemetryHttpSuccessCount = 0;
static volatile uint32_t telemetryHttpFailureCount = 0;
static volatile uint32_t telemetryHttpLastSequence = 0;
static volatile uint32_t telemetryHttpLastDurationMs = 0;
static volatile int32_t telemetryHttpLastCode = 0;
static volatile bool telemetryHttpSocketConnected = false;
static volatile bool telemetryHttpPauseRequested = true;
static volatile uint8_t telemetryHttpConsecutiveFailures = 0;
static volatile uint32_t telemetryHttpLastFailureMs = 0;
static volatile uint32_t telemetryHttpConnectionCount = 0;
static volatile uint32_t telemetryHttpReuseCount = 0;
static volatile uint32_t telemetryHttpResponseErrorCount = 0;
static volatile uint16_t telemetryHttpLocalPort = 0;
static volatile uint32_t telemetryHttpCircuitBreakCount = 0;
static volatile uint32_t telemetryHttpCircuitOpenUntilMs = 0;
static volatile uint8_t telemetryHttpFailureWindowCount = 0;
static volatile uint8_t telemetryHttpStateCode = 0;
static volatile uint32_t telemetryHttpServiceMaxUs = 0;
static volatile uint32_t telemetryHttpCancelledCount = 0;
static volatile int8_t telemetryHttpOwnerCore = -1;
static volatile uint32_t telemetryHttpHealthIsolationCount = 0;
static volatile uint32_t telemetryHttpPreventiveRotationCount = 0;
#define TELEMETRY_HTTP_PERIOD_MS 1000UL
#define TELEMETRY_HTTP_CONNECT_TIMEOUT_MS 2000UL
#define TELEMETRY_HTTP_TRANSACTION_TIMEOUT_MS 5000UL
#define TELEMETRY_HTTP_CIRCUIT_WINDOW_MS 60000UL
#define TELEMETRY_HTTP_CIRCUIT_OPEN_MS 90000UL
#define TELEMETRY_HTTP_HEALTH_HOLDOFF_MS 60000UL
#define TELEMETRY_HTTP_MAX_REQUESTS_PER_SOCKET 60UL
QueueHandle_t connectionSnapshotQueue = NULL;  // Core 0 -> Core 1
static uint32_t core1TelemetrySequence = 0;

// v3.9.151 strict core boundary. Core 1 is the only producer of these records;
// Core 0 only peeks/drains them with zero wait. Neither direction can block the
// other core. Sensor debug text is formatted on Core 1, but the physical UART
// is written only by Core 0 so Arduino Serial's internal lock can never couple
// a slow sensor path back into Blynk/WiFi service.
struct SensorDiagnosticsSnapshot {
  uint32_t capturedAtMs;
  uint32_t heartbeat;
  uint32_t ppsCount;
  uint32_t ppsAgeMs;
  uint32_t maxSensorCycleMs;
  uint32_t maxGpsFeedMs;
  uint32_t maxI2cPathMs;
  uint32_t maxPpsLedResponseMs;
  uint32_t lastI2cDevice;
  uint32_t lastI2cOperation;
  uint32_t i2cErrorCount;
  uint32_t maxAht21Ms;
  uint32_t maxEns160Ms;
  uint32_t maxBmi160Ms;
  uint32_t stageMaxEnsMs;
  uint32_t stageMaxInmpMs;
  uint32_t stageMaxCoMs;
  uint32_t stageMaxGpsMs;
  uint32_t stageMaxInavMs;
  uint32_t stageMaxOledMs;
  uint32_t stageWarnCount;
  uint32_t dataMutexCalls;
  uint32_t dataMutexTimeouts;
  uint32_t dataMutexMaxWaitUs;
  uint32_t ledcLockCalls;
  uint32_t ledcLockTimeouts;
  uint32_t ledcLockMaxWaitUs;
  uint32_t i2cProbeFailCount;
  uint32_t i2cRecoveryCount;
  uint8_t  i2cSdaLevel;
  uint8_t  i2cSclLevel;
  uint8_t  sensorState;
  uint8_t  coPhase;
  uint8_t  coDuty;
  float    coHeaterV;
};

struct SensorLogRecord { char text[192]; };
enum : uint8_t { BUZZER_CMD_TONE = 1, BUZZER_CMD_PATTERN3 = 2 };
struct BuzzerCommand {
  uint8_t type;
  uint16_t frequency;
  uint16_t durationMs;
  uint16_t pauseMs;
};
QueueHandle_t sensorDiagnosticsQueue = NULL;
QueueHandle_t sensorLogQueue = NULL;
QueueHandle_t buzzerCommandQueue = NULL;
static SensorDiagnosticsSnapshot core0SensorDiag = {};
static bool core0SensorDiagValid = false;
static volatile uint32_t sensorLogDropped = 0;
static char core0EngineeringMessage[128] = "Booting...";
static uint32_t core0EngineeringMessageAtMs = 0;

// LEDC counters are defined with the LEDC helpers later in the file. Forward
// declarations let the Core-1 diagnostic snapshot capture them here.
extern volatile uint32_t ledcLockTakeCalls;
extern volatile uint32_t ledcLockTimeouts;
extern volatile uint32_t ledcLockMaxWaitUs;

static void sensorLogEnqueue(const char* text)
{
  if (sensorLogQueue == NULL || text == NULL) { ++sensorLogDropped; return; }
  SensorLogRecord record = {};
  strncpy(record.text, text, sizeof(record.text) - 1);
  if (xQueueSend(sensorLogQueue, &record, 0) != pdTRUE) ++sensorLogDropped;
}

static void sensorLogPrintf(const char* fmt, ...)
{
  char text[192];
  va_list args; va_start(args, fmt);
  vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);
  sensorLogEnqueue(text);
}

void isolatedDebugPrintln(const char* text)
{
  if (xPortGetCoreID() == 1) sensorLogEnqueue(text);
  else Serial.println(text);
}

void isolatedDebugPrintf(const char* fmt, ...)
{
  char text[192];
  va_list args; va_start(args, fmt);
  vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);
  if (xPortGetCoreID() == 1) sensorLogEnqueue(text);
  else Serial.print(text);
}

static void core0DrainSensorLog()
{
  if (sensorLogQueue == NULL) return;
  SensorLogRecord record;
  // One bounded record per loop pass keeps logging subordinate to Blynk.run().
  if (xQueueReceive(sensorLogQueue, &record, 0) == pdTRUE) Serial.print(record.text);
}

static void core0RefreshSensorDiagnostics()
{
  SensorDiagnosticsSnapshot newest;
  if (sensorDiagnosticsQueue != NULL &&
      xQueuePeek(sensorDiagnosticsQueue, &newest, 0) == pdTRUE) {
    core0SensorDiag = newest;
    core0SensorDiagValid = true;
  }
}
// Core-1-local SensorData guard. Blynk/Core 0 never takes this mutex; it reads
// immutable queue copies instead. Timed waits remain as protection against an
// accidental future re-entrant sensor path, not as cross-core synchronization.
volatile uint32_t dataMutexTakeCalls = 0;
volatile uint32_t dataMutexTakeTimeouts = 0;
volatile uint32_t dataMutexMaxWaitUs = 0;

static bool dataMutexTake(uint32_t timeoutMs)
{
  if (dataMutex == NULL) return false;
  const uint32_t startUs = micros();
  ++dataMutexTakeCalls;
  const bool ok = (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
  const uint32_t waitedUs = micros() - startUs;
  if (waitedUs > dataMutexMaxWaitUs) dataMutexMaxWaitUs = waitedUs;
  if (!ok) ++dataMutexTakeTimeouts;
  return ok;
}

// Core-0-only cache used by Blynk publishers. Core 1 owns sensor I/O and
// publishes complete SensorData values through a one-slot overwrite queue.
// A network callback only peeks the queue and therefore never locks sensor
// state, invokes a sensor driver, or waits for Core 1.
static SensorData blynkLastSensorSnapshot;
static bool blynkLastSensorSnapshotValid = false;
static uint32_t blynkSensorSnapshotFreshCopies = 0;
static uint32_t blynkSensorSnapshotCachedUses = 0;

static bool blynkGetSensorSnapshot(SensorData& snapshot)
{
  SensorData newest;
  if (sensorSnapshotQueue != NULL && xQueuePeek(sensorSnapshotQueue, &newest, 0) == pdTRUE) {
    blynkLastSensorSnapshot = newest;
    blynkLastSensorSnapshotValid = true;
    ++blynkSensorSnapshotFreshCopies;
  } else if (blynkLastSensorSnapshotValid) {
    ++blynkSensorSnapshotCachedUses;
  }

  if (!blynkLastSensorSnapshotValid) return false;
  snapshot = blynkLastSensorSnapshot;
  // Connection state belongs to Core 0 and is overlaid only after the immutable
  // Core-1 snapshot has crossed the queue. Core 0 never writes back into sd.
  if (WiFi.status() == WL_CONNECTED) snapshot.status_flags |= STATUS_WIFI_OK;
  else                               snapshot.status_flags &= ~STATUS_WIFI_OK;
  if (core0EngineeringMessageAtMs != 0 &&
      (snapshot.eng_msg_at_ms == 0 ||
       (int32_t)(core0EngineeringMessageAtMs - snapshot.eng_msg_at_ms) >= 0)) {
    strncpy(snapshot.eng_msg, core0EngineeringMessage, sizeof(snapshot.eng_msg) - 1);
    snapshot.eng_msg[sizeof(snapshot.eng_msg) - 1] = '\0';
    snapshot.eng_msg_at_ms = core0EngineeringMessageAtMs;
  }
  return true;
}

// Core-1-only serializer. Both strings are constructed from one immutable
// SensorData copy and one captured record key. Core 0 receives a complete
// value-copy; it never formats, edits or independently refreshes either half.
static void core1PublishTelemetryPair(const SensorData& snapshot)
{
  if (telemetryPairQueue == NULL) return;

  Core0ConnectionSnapshot connection = {};
  if (connectionSnapshotQueue != NULL) {
    (void)xQueuePeek(connectionSnapshotQueue, &connection, 0);
  }

  Core1TelemetryPair pair = {};
  pair.sequence = ++core1TelemetrySequence;
  pair.deviceMs = millis();
  pair.gpsUnix = snapshot.gps_unix_time;
  uint32_t flags = snapshot.status_flags;
  if (connection.wifiConnected) flags |= STATUS_WIFI_OK;
  else                          flags &= ~STATUS_WIFI_OK;

  snprintf(pair.v61, sizeof(pair.v61),
           "%ld,%lu,%.7f,%.7f,%.1f,%.2f,%d,%.2f,%.1f,%.2f,%u,%d,%d,%.1f,%.0f,%.1f,%.2f,%.1f,%.3f",
           pair.gpsUnix, (unsigned long)pair.deviceMs,
           snapshot.lat, snapshot.lng,
           snapshot.sats, snapshot.hdop, snapshot.gps_fix ? 1 : 0,
           snapshot.co_ppm, snapshot.co_raw, snapshot.co_heater_v, snapshot.co_phase,
           snapshot.co_fault ? 1 : 0, snapshot.co_no_sensor ? 1 : 0,
           snapshot.tvoc_ppb, snapshot.eco2_ppm, snapshot.aqi, snapshot.uvi,
           snapshot.sound_db, snapshot.dust_mg);

  snprintf(pair.v62, sizeof(pair.v62),
           "%ld,%lu,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.7f,%.7f,%.1f,%.2f,%.1f,%lu,%ld,%u",
           pair.gpsUnix, (unsigned long)pair.deviceMs,
           snapshot.temp, snapshot.hum, snapshot.ens_fault ? 1 : 0,
           snapshot.angle[0], snapshot.angle[1], snapshot.angle[2], snapshot.imu_temp,
           snapshot.imu_ok ? 1 : 0, snapshot.pps_locked ? 1 : 0, snapshot.inav_state,
           snapshot.inav_lat, snapshot.inav_lng, snapshot.inav_alt,
           snapshot.inav_speed_mps, snapshot.inav_heading_deg,
           (unsigned long)flags, (long)connection.rssi,
           (unsigned)connection.wifiQuality);

  xQueueOverwrite(telemetryPairQueue, &pair);
}

enum TelemetryNetworkState : uint8_t {
  TELEMETRY_NET_IDLE = 0,
  TELEMETRY_NET_CONNECTING = 1,
  TELEMETRY_NET_SENDING = 2,
  TELEMETRY_NET_READING = 3,
  TELEMETRY_NET_CIRCUIT_OPEN = 4
};

struct TelemetryNetworkContext {
  int sock = -1;
  TelemetryNetworkState state = TELEMETRY_NET_IDLE;
  uint32_t stateStartedAt = 0;
  uint32_t requestStartedAt = 0;
  uint32_t nextAttemptAt = 0;
  uint32_t completedSequence = 0;
  uint32_t failureWindowStartedAt = 0;
  uint32_t circuitOpenUntil = 0;
  uint8_t failureWindowCount = 0;
  uint8_t consecutiveFailures = 0;
  uint32_t requestsOnSocket = 0;
  uint16_t activeLocalPort = 0;
  Core1TelemetryPair inFlight = {};
  char body[520] = {};
  size_t bodyLength = 0;
  char request[896] = {};
  size_t requestLength = 0;
  size_t requestSent = 0;
  char response[768] = {};
  size_t responseLength = 0;
};

static TelemetryNetworkContext telemetryNet;
static bool telemetryHttpHealthPauseLatched = false;
static bool telemetryHttpHealthThreatActive = false;
static uint32_t telemetryHttpHealthResumeNotBeforeMs = 0;

static void telemetryNetworkPublishState()
{
  telemetryHttpStateCode = (uint8_t)telemetryNet.state;
  telemetryHttpSocketConnected = telemetryNet.sock >= 0;
  telemetryHttpLocalPort = telemetryNet.sock >= 0 ? telemetryNet.activeLocalPort : 0;
  telemetryHttpCircuitOpenUntilMs = telemetryNet.circuitOpenUntil;
  telemetryHttpFailureWindowCount = telemetryNet.failureWindowCount;
  telemetryHttpConsecutiveFailures = telemetryNet.consecutiveFailures;
}

static void telemetryNetworkCloseSocket()
{
  if (telemetryNet.sock >= 0) {
    lwip_close(telemetryNet.sock);
  }
  telemetryNet.sock = -1;
  telemetryNet.activeLocalPort = 0;
  telemetryNet.requestsOnSocket = 0;
  telemetryHttpSocketConnected = false;
  telemetryHttpLocalPort = 0;
}

static uint16_t telemetryNetworkLocalPort(int sock)
{
  struct sockaddr_in localAddress = {};
  socklen_t localLength = sizeof(localAddress);
  if (sock >= 0 &&
      lwip_getsockname(sock, reinterpret_cast<struct sockaddr *>(&localAddress),
                       &localLength) == 0) {
    return ntohs(localAddress.sin_port);
  }
  return 0;
}

static bool telemetryNetworkAppendJsonString(char *output, size_t capacity,
                                             size_t& used, const char *value)
{
  if (used + 1U >= capacity) return false;
  output[used++] = '"';
  for (const unsigned char *p = reinterpret_cast<const unsigned char *>(value);
       p != nullptr && *p; ++p) {
    if (*p == '"' || *p == '\\') {
      if (used + 2U >= capacity) return false;
      output[used++] = '\\';
      output[used++] = (char)*p;
    } else if (*p < 0x20U) {
      if (used + 6U >= capacity) return false;
      const int written = snprintf(output + used, capacity - used,
                                   "\\u%04x", (unsigned)*p);
      if (written != 6) return false;
      used += 6U;
    } else {
      if (used + 1U >= capacity) return false;
      output[used++] = (char)*p;
    }
  }
  if (used + 1U >= capacity) return false;
  output[used++] = '"';
  output[used] = '\0';
  return true;
}

static bool telemetryNetworkBuildRequest(const Core1TelemetryPair& pair)
{
  size_t used = 0;
  telemetryNet.body[used++] = '[';
  if (!telemetryNetworkAppendJsonString(telemetryNet.body,
                                        sizeof(telemetryNet.body), used,
                                        pair.v61)) return false;
  if (used + 1U >= sizeof(telemetryNet.body)) return false;
  telemetryNet.body[used++] = ',';
  if (!telemetryNetworkAppendJsonString(telemetryNet.body,
                                        sizeof(telemetryNet.body), used,
                                        pair.v62)) return false;
  if (used + 2U > sizeof(telemetryNet.body)) return false;
  telemetryNet.body[used++] = ']';
  telemetryNet.body[used] = '\0';
  telemetryNet.bodyLength = used;

  const int headerLength = snprintf(
      telemetryNet.request, sizeof(telemetryNet.request),
      "PUT /%s/telemetry/coherent HTTP/1.1\r\n"
      "Host: %s:%u\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %u\r\n"
      "Connection: keep-alive\r\n\r\n",
      BLYNK_AUTH, BLYNK_SERVER, (unsigned)BLYNK_PORT,
      (unsigned)telemetryNet.bodyLength);
  if (headerLength <= 0 ||
      (size_t)headerLength + telemetryNet.bodyLength >=
          sizeof(telemetryNet.request)) return false;

  memcpy(telemetryNet.request + headerLength, telemetryNet.body,
         telemetryNet.bodyLength);
  telemetryNet.requestLength = (size_t)headerLength + telemetryNet.bodyLength;
  telemetryNet.requestSent = 0;
  telemetryNet.responseLength = 0;
  telemetryNet.response[0] = '\0';
  return true;
}

static void telemetryNetworkMarkSocketUse()
{
  if (telemetryNet.requestsOnSocket > 0) {
    ++telemetryHttpReuseCount;
  }
  ++telemetryNet.requestsOnSocket;
}

static void telemetryNetworkRecordFailure(uint32_t now, int32_t code,
                                          bool responseError)
{
  telemetryHttpLastDurationMs = now - telemetryNet.requestStartedAt;
  telemetryHttpLastCode = code;
  telemetryHttpLastFailureMs = now;
  ++telemetryHttpFailureCount;
  if (responseError) ++telemetryHttpResponseErrorCount;
  if (telemetryNet.consecutiveFailures < 5U) {
    ++telemetryNet.consecutiveFailures;
  }

  if (telemetryNet.failureWindowStartedAt == 0 ||
      now - telemetryNet.failureWindowStartedAt >=
          TELEMETRY_HTTP_CIRCUIT_WINDOW_MS) {
    telemetryNet.failureWindowStartedAt = now;
    telemetryNet.failureWindowCount = 1;
  } else if (telemetryNet.failureWindowCount < 255U) {
    ++telemetryNet.failureWindowCount;
  }

  telemetryNetworkCloseSocket();
  telemetryNet.state = TELEMETRY_NET_IDLE;
  if (telemetryNet.failureWindowCount >= 3U) {
    telemetryNet.circuitOpenUntil = now + TELEMETRY_HTTP_CIRCUIT_OPEN_MS;
    telemetryNet.state = TELEMETRY_NET_CIRCUIT_OPEN;
    ++telemetryHttpCircuitBreakCount;
    telemetryNet.failureWindowStartedAt = 0;
    telemetryNet.failureWindowCount = 0;
    telemetryNet.nextAttemptAt = telemetryNet.circuitOpenUntil;
  } else {
    uint32_t retryMs = 1000UL << telemetryNet.consecutiveFailures;
    if (retryMs > 30000UL) retryMs = 30000UL;
    telemetryNet.nextAttemptAt = now + retryMs;
  }
  telemetryNetworkPublishState();
}

static void telemetryNetworkRecordSuccess(uint32_t now)
{
  telemetryNet.completedSequence = telemetryNet.inFlight.sequence;
  telemetryHttpLastSequence = telemetryNet.inFlight.sequence;
  telemetryHttpLastDurationMs = now - telemetryNet.requestStartedAt;
  telemetryHttpLastCode = 200;
  ++telemetryHttpSuccessCount;
  telemetryNet.consecutiveFailures = 0;
  if (telemetryNet.failureWindowStartedAt != 0 &&
      now - telemetryNet.failureWindowStartedAt >=
          TELEMETRY_HTTP_CIRCUIT_WINDOW_MS) {
    telemetryNet.failureWindowStartedAt = 0;
    telemetryNet.failureWindowCount = 0;
  }
  telemetryNet.state = TELEMETRY_NET_IDLE;
  telemetryNet.nextAttemptAt = now + TELEMETRY_HTTP_PERIOD_MS;
  telemetryNetworkPublishState();
}

static bool telemetryNetworkSocketUsable()
{
  if (telemetryNet.sock < 0) return false;
  uint8_t probe = 0;
  errno = 0;
  const int result = lwip_recv(telemetryNet.sock, &probe, 1,
                               MSG_DONTWAIT | MSG_PEEK);
  if (result == 0) {
    telemetryNetworkCloseSocket();
    return false;
  }
  if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    telemetryNetworkCloseSocket();
    return false;
  }
  if (result > 0) {
    // No response bytes are legal between requests. Retaining such a socket
    // would desynchronise the next HTTP response, so rotate it once.
    telemetryNetworkCloseSocket();
    return false;
  }
  return true;
}

static bool telemetryNetworkStartConnect(uint32_t now)
{
  IPAddress ip;
  if (!ip.fromString(BLYNK_SERVER)) return false;

  const int sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) return false;
  const int flags = lwip_fcntl(sock, F_GETFL, 0);
  if (flags < 0 || lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
    lwip_close(sock);
    return false;
  }

  struct sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_port = htons(BLYNK_PORT);
  address.sin_addr.s_addr = (uint32_t)ip;

  telemetryNet.sock = sock;
  telemetryNet.stateStartedAt = now;
  errno = 0;
  const int result = lwip_connect(
      sock, reinterpret_cast<struct sockaddr *>(&address), sizeof(address));
  if (result == 0) {
    telemetryNet.state = TELEMETRY_NET_SENDING;
    telemetryNet.activeLocalPort = telemetryNetworkLocalPort(sock);
    ++telemetryHttpConnectionCount;
    telemetryNetworkMarkSocketUse();
    return true;
  }
  if (errno == EINPROGRESS || errno == EALREADY || errno == EWOULDBLOCK) {
    telemetryNet.state = TELEMETRY_NET_CONNECTING;
    return true;
  }

  telemetryNetworkCloseSocket();
  return false;
}

// Core-0-only bounded service. No operation waits: connect readiness uses
// select(tv=0), and send/recv always use MSG_DONTWAIT. Core 1 never calls it.
static void telemetryNetworkService()
{
  const uint32_t serviceStartedUs = micros();
  const uint32_t now = millis();
  telemetryHttpOwnerCore = (int8_t)xPortGetCoreID();

  if (WiFi.status() != WL_CONNECTED) {
    telemetryNetworkCloseSocket();
    telemetryNet.state = TELEMETRY_NET_IDLE;
    telemetryNet.failureWindowStartedAt = 0;
    telemetryNet.failureWindowCount = 0;
    telemetryNet.circuitOpenUntil = 0;
    telemetryNet.consecutiveFailures = 0;
    telemetryNet.nextAttemptAt = now + 1000UL;
  } else if (telemetryHttpPauseRequested) {
    // v3.9.209: isolation means zero HTTP PCBs, not merely no new request.
    // v3.9.208 left an idle persistent HTTP socket alive precisely while the
    // Blynk socket was being rotated, so the suspect shared receive path was
    // never fully cleared.
    if (telemetryNet.sock >= 0) {
      const bool active = telemetryNet.state == TELEMETRY_NET_CONNECTING ||
          telemetryNet.state == TELEMETRY_NET_SENDING ||
          telemetryNet.state == TELEMETRY_NET_READING;
      telemetryNetworkCloseSocket();
      telemetryNet.state = TELEMETRY_NET_IDLE;
      telemetryNet.nextAttemptAt = now + 1000UL;
      if (active) ++telemetryHttpCancelledCount;
    }
  } else if (telemetryNet.circuitOpenUntil != 0 &&
             (int32_t)(now - telemetryNet.circuitOpenUntil) < 0) {
    telemetryNetworkCloseSocket();
    telemetryNet.state = TELEMETRY_NET_CIRCUIT_OPEN;
  } else {
    if (telemetryNet.circuitOpenUntil != 0) {
      telemetryNet.circuitOpenUntil = 0;
      telemetryNet.state = TELEMETRY_NET_IDLE;
      telemetryNet.nextAttemptAt = now;
    }

    if (telemetryNet.state == TELEMETRY_NET_IDLE &&
        (int32_t)(now - telemetryNet.nextAttemptAt) >= 0 &&
        telemetryPairQueue != NULL) {
      if (telemetryNet.sock >= 0 &&
          telemetryNet.requestsOnSocket >=
              TELEMETRY_HTTP_MAX_REQUESTS_PER_SOCKET) {
        telemetryNetworkCloseSocket();
        ++telemetryHttpPreventiveRotationCount;
      }
      Core1TelemetryPair pair = {};
      if (xQueuePeek(telemetryPairQueue, &pair, 0) == pdTRUE &&
          pair.sequence != 0 &&
          pair.sequence != telemetryNet.completedSequence) {
        telemetryNet.inFlight = pair;
        telemetryNet.requestStartedAt = now;
        if (!telemetryNetworkBuildRequest(pair)) {
          telemetryNetworkRecordFailure(now, -12, true);
        } else if (telemetryNetworkSocketUsable()) {
          telemetryNetworkMarkSocketUse();
          telemetryNet.state = TELEMETRY_NET_SENDING;
          telemetryNet.stateStartedAt = now;
        } else if (!telemetryNetworkStartConnect(now)) {
          telemetryNetworkRecordFailure(now, -1, false);
        }
      }
    } else if (telemetryNet.state == TELEMETRY_NET_CONNECTING) {
      fd_set writeSet;
      fd_set errorSet;
      FD_ZERO(&writeSet);
      FD_ZERO(&errorSet);
      FD_SET(telemetryNet.sock, &writeSet);
      FD_SET(telemetryNet.sock, &errorSet);
      struct timeval zeroWait = {0, 0};
      const int ready = lwip_select(telemetryNet.sock + 1, nullptr, &writeSet,
                                    &errorSet, &zeroWait);
      if (ready > 0) {
        int socketError = 0;
        socklen_t errorLength = sizeof(socketError);
        if (lwip_getsockopt(telemetryNet.sock, SOL_SOCKET, SO_ERROR,
                            &socketError, &errorLength) == 0 &&
            socketError == 0 && FD_ISSET(telemetryNet.sock, &writeSet)) {
          telemetryNet.activeLocalPort =
              telemetryNetworkLocalPort(telemetryNet.sock);
          ++telemetryHttpConnectionCount;
          telemetryNetworkMarkSocketUse();
          telemetryNet.state = TELEMETRY_NET_SENDING;
          telemetryNet.stateStartedAt = now;
        } else {
          telemetryNetworkRecordFailure(now, -1, false);
        }
      } else if (ready < 0 ||
                 now - telemetryNet.stateStartedAt >=
                     TELEMETRY_HTTP_CONNECT_TIMEOUT_MS) {
        telemetryNetworkRecordFailure(now, -1, false);
      }
    } else if (telemetryNet.state == TELEMETRY_NET_SENDING) {
      if (now - telemetryNet.requestStartedAt >=
          TELEMETRY_HTTP_TRANSACTION_TIMEOUT_MS) {
        telemetryNetworkRecordFailure(now, -11, false);
      } else {
        errno = 0;
        const int sent = lwip_send(
            telemetryNet.sock,
            telemetryNet.request + telemetryNet.requestSent,
            telemetryNet.requestLength - telemetryNet.requestSent,
            MSG_DONTWAIT);
        if (sent > 0) {
          telemetryNet.requestSent += (size_t)sent;
          if (telemetryNet.requestSent == telemetryNet.requestLength) {
            telemetryNet.state = TELEMETRY_NET_READING;
            telemetryNet.stateStartedAt = now;
          }
        } else if (sent == 0 ||
                   (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                    errno != EINTR)) {
          telemetryNetworkRecordFailure(now, -1, false);
        }
      }
    } else if (telemetryNet.state == TELEMETRY_NET_READING) {
      if (now - telemetryNet.requestStartedAt >=
          TELEMETRY_HTTP_TRANSACTION_TIMEOUT_MS) {
        telemetryNetworkRecordFailure(now, -11, false);
      } else if (telemetryNet.responseLength + 1U >= sizeof(telemetryNet.response)) {
        telemetryNetworkRecordFailure(now, -12, true);
      } else {
        errno = 0;
        const int received = lwip_recv(
            telemetryNet.sock,
            telemetryNet.response + telemetryNet.responseLength,
            sizeof(telemetryNet.response) - telemetryNet.responseLength - 1U,
            MSG_DONTWAIT);
        if (received > 0) {
          telemetryNet.responseLength += (size_t)received;
          telemetryNet.response[telemetryNet.responseLength] = '\0';
          char *body = strstr(telemetryNet.response, "\r\n\r\n");
          if (body != nullptr) {
            body += 4;
            const size_t bodyBytes = telemetryNet.responseLength -
                (size_t)(body - telemetryNet.response);
            if (bodyBytes >= 2U) {
              const bool statusOk =
                  strncmp(telemetryNet.response, "HTTP/1.1 200", 12) == 0 ||
                  strncmp(telemetryNet.response, "HTTP/1.0 200", 12) == 0;
              if (statusOk && body[0] == 'O' && body[1] == 'K') {
                telemetryNetworkRecordSuccess(now);
              } else {
                telemetryNetworkRecordFailure(now, -12, true);
              }
            }
          }
        } else if (received == 0 ||
                   (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                    errno != EINTR)) {
          telemetryNetworkRecordFailure(now, -1, false);
        }
      }
    }
  }

  telemetryNetworkPublishState();
  const uint32_t elapsedUs = micros() - serviceStartedUs;
  if (elapsedUs > telemetryHttpServiceMaxUs) {
    telemetryHttpServiceMaxUs = elapsedUs;
  }
}

// --- CO STATE ---------------------------------------------------------------
byte          co_phase          = 0;    // 0=measure, 1=heat
unsigned long co_phase_start    = 0;
float         sens_val          = 0;   // EMA of CO_ADC_PIN
float         sens_val_last     = 0;
float         last_CO_ppm       = 0;
volatile float lastCoHeaterV    = 0.0f;
byte          co_duty           = 255; // closed-loop duty output
float         sensor_base_resistance_kOhm;
float         sensor_100ppm_CO_resistance_kOhm;

// ─── IMU — BMI160 fusion state (v2.4, replaces JY-901) ──────────────────────
// angle[3] = Roll, Pitch, Yaw (deg). Roll/Pitch use the complementary
// accelerometer+gyro filter. Yaw is the relative BMI160 Z-axis gyro heading
// (continuous, 0..360 deg) so it remains useful even before GPS acquires a fix.
// It is not an absolute compass heading because BMI160 has no magnetometer.
// imuT = BMI160 die temperature (°C).
float         angle[3]         = {0,0,0};
float         imuT             = 0;
float         imuYawDeg        = 0.0f;
unsigned long lastBmiOkMillis  = 0;   // last successful BMI160 read, for stuck detection
unsigned long lastFusionMillis = 0;   // for complementary filter dt
unsigned long lastImuYawMillis = 0;   // relative gyro-Z yaw integration timestamp

// ─── ENS160 / AHT2x state ────────────────────────────────────────────────────
int           ens_fail_count = 0;
unsigned long lastENSattempt = 0;
bool          ensReady       = false;  // true after first successful init
uint8_t       ens_i2c_fail_streak = 0;
uint8_t       bmi_i2c_fail_streak = 0;
float         hum_prev = 20.0f, temp_prev = 20.0f;

// ─── MISC STATE ──────────────────────────────────────────────────────────────
unsigned long lastGPSdata   = 0;
unsigned long lastGPSbeep   = 0;
unsigned long lastWiFiCheck = 0;
long          countReset    = 0;
bool          prevZeroVal   = false;
float         SatGPS = 0, HDOP = 0;

// ─── RESET-CAUSE DIAGNOSTICS ────────────────────────────────────────────────
// Additive diagnostic only: records why the ESP32 last restarted.
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
const char* bootResetReasonText = "UNKNOWN";

// ─── SENSOR HEALTH DIAGNOSTICS — additive reliability layer ───────────────────
// These counters only detect persistent invalid/no-data conditions. They never
// stop sensorTask and never prevent the other sensors from being serviced.
// Existing sensor-specific fault/retry logic remains unchanged.
unsigned int inmpFaultCount = 0;
unsigned int uvFaultCount   = 0;
unsigned int dustFaultCount = 0;
bool inmpFaultReported = false;
bool uvFaultReported   = false;
bool dustFaultReported = false;
bool inmpLastReadHadData = false;
unsigned long lastInmpHealthMsg = 0;
unsigned long lastUvHealthMsg   = 0;
unsigned long lastDustHealthMsg = 0;

#define SENSOR_HEALTH_FAIL_LIMIT 5
#define SENSOR_HEALTH_MSG_INTERVAL 60000UL

// Report a persistent fault without ever stopping the sensor task. The existing
// engineering-message path (Blynk V19) is deliberately reused so no new Blynk
// data channel is required and no existing virtual-pin mapping changes.
void sensorHealthMsg(const char* sensor, const char* reason, unsigned long& lastMsg)
{
  unsigned long now = millis();
  if (now - lastMsg >= SENSOR_HEALTH_MSG_INTERVAL || lastMsg == 0) {
    lastMsg = now;
    engMsgf("%s FAULT: %s — other sensors continue", sensor, reason);
  }
}

// ─── ASYNCHRONOUS BUZZER STATE ───────────────────────────────────────────────
// Additive reliability change: buzzerTone() starts a tone and returns immediately.
// buzzerService() advances the tone without blocking either FreeRTOS task.
volatile bool buzzerActive = false;
volatile uint16_t buzzerActiveFrequency = 0;
volatile unsigned long buzzerStopMillis = 0;

#define BUZZER_PATTERN_MAX_STEPS 3
struct BuzzerPatternStep {
  uint16_t frequency;
  uint16_t durationMs;
  uint16_t pauseMs;
};
BuzzerPatternStep buzzerPattern[BUZZER_PATTERN_MAX_STEPS];
volatile uint8_t buzzerPatternCount = 0;
volatile uint8_t buzzerPatternIndex = 0;
volatile bool buzzerPatternRunning = false;
volatile bool buzzerLEDCReady = false; // v3.9.4: declared before buzzerTone() so Arduino/C++ sees it.
volatile unsigned long buzzerPatternNextMillis = 0;

// ─── LONG-TERM HEALTH MONITORING ─────────────────────────────────────────────
// Sensor-task heartbeat is updated once per completed sensor cycle.  The
// application watchdog below uses it as an additional diagnostic, while the
// ESP task watchdog protects against a genuinely stuck sensor task.
volatile uint32_t sensorTaskHeartbeat = 0;
unsigned long lastHealthReport = 0;
unsigned long lastHeartbeatChangeMillis = 0;
uint32_t lastHealthHeartbeat = 0;
#define SENSOR_TASK_RECOVERY_REQUEST_MS 8000UL
#define SENSOR_TASK_RECOVERY_ACK_TIMEOUT_MS 8000UL
volatile uint32_t sensorRecoveryRequestSequence = 0;
volatile uint32_t sensorRecoveryAckSequence = 0;
volatile uint32_t sensorRecoveryCount = 0;
bool sensorRecoveryAwaitingAck = false;
unsigned long sensorRecoveryRequestedAt = 0;

// ─── CORE 1 STALL DIAGNOSTICS — additive v3.9.18 ─────────────────────────────
// The ESP32 remains alive when Blynk stops updating, so diagnose the Arduino
// loop itself rather than assuming a full-system failure.  The monitor task runs
// independently on Core 1 and only reports prolonged lack of loop progress.
#define CORE1_STALL_WARN_MS 500UL
#define CORE1_STAGE_WARN_INTERVAL_MS 1000UL
#define CORE0_HARD_STALL_RESET_MS 30000UL
#define CORE0_STALL_RTC_MAGIC 0x43305352UL  // "C0SR"

struct Core0StallRecoveryRecord {
  uint32_t magic;
  uint32_t resetCount;
  uint32_t detectedAtMs;
  uint32_t progressAgeMs;
  uint16_t stageId;
  uint16_t reserved;
};

RTC_NOINIT_ATTR Core0StallRecoveryRecord core0StallRecoveryRecord = {};

volatile uint32_t core1LastProgressUs = 0;
volatile uint32_t core1StageStartUs = 0;
volatile uint32_t core1MaxStageMs = 0;
volatile uint32_t core1StallCount = 0;
volatile uint16_t core1StageId = 0;
volatile bool core1StallReported = false;
unsigned long core1LastStallReportMillis = 0;
volatile uint32_t core1StallStartUs = 0;
volatile bool core1WifiConnectedSnapshot = false;
volatile bool core1BlynkConnectedSnapshot = false;

// Stage IDs are deliberately small so the most recent stage can also be saved
// in the existing RTC checkpoint without storing a long string there.
enum : uint16_t {
  CORE1_STAGE_IDLE = 0,
  CORE1_STAGE_BUZZER = 1,
  CORE1_STAGE_WIFI_SERVICE = 2,
  CORE1_STAGE_TCP_PROBE = 3,
  CORE1_STAGE_BLYNK_HANDSHAKE = 4,
  CORE1_STAGE_BLYNK_RUN = 5,
  CORE1_STAGE_BLYNK_HEARTBEAT = 6,
  CORE1_STAGE_BLYNK_TIMER = 7,
  CORE1_STAGE_BLYNK_TX = 8,
  CORE1_STAGE_HEALTH = 9,
  CORE1_STAGE_RTC = 10,
  CORE1_STAGE_MUTEX = 11,
  CORE1_STAGE_BLYNK_TX_CALL = 12,
  CORE1_STAGE_BLYNK_MAP_TX_CALL = 13
};

static const char* core1StageText(uint16_t stage)
{
  switch (stage) {
    case CORE1_STAGE_BUZZER:           return "BUZZER";
    case CORE1_STAGE_WIFI_SERVICE:     return "WIFI_SERVICE";
    case CORE1_STAGE_TCP_PROBE:        return "TCP_PROBE";
    case CORE1_STAGE_BLYNK_HANDSHAKE:  return "BLYNK_HANDSHAKE";
    case CORE1_STAGE_BLYNK_RUN:        return "BLYNK_RUN";
    case CORE1_STAGE_BLYNK_HEARTBEAT:  return "BLYNK_HEARTBEAT";
    case CORE1_STAGE_BLYNK_TIMER:      return "BLYNK_TIMER";
    case CORE1_STAGE_BLYNK_TX:         return "BLYNK_TX";
    case CORE1_STAGE_HEALTH:           return "HEALTH";
    case CORE1_STAGE_RTC:              return "RTC";
    case CORE1_STAGE_MUTEX:            return "MUTEX";
    case CORE1_STAGE_BLYNK_TX_CALL:    return "BLYNK_TX_CALL";
    case CORE1_STAGE_BLYNK_MAP_TX_CALL:return "BLYNK_MAP_TX_CALL";
    default:                           return "IDLE";
  }
}

static inline void core1StageBegin(uint16_t stage)
{
  uint32_t nowUs = micros();
  core1StageId = stage;
  core1StageStartUs = nowUs;
  core1LastProgressUs = nowUs;
}

static inline void core1StageEnd()
{
  uint32_t nowUs = micros();
  uint32_t startUs = core1StageStartUs;
  uint32_t elapsedMs = (uint32_t)((nowUs - startUs) / 1000UL);
  if (elapsedMs > core1MaxStageMs) core1MaxStageMs = elapsedMs;
  core1StageId = CORE1_STAGE_IDLE;
  core1StageStartUs = nowUs;
  core1LastProgressUs = nowUs;
}

void core1StallMonitorTask(void* pvParam)
{
  (void)pvParam;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!setupDone) continue;

    // Ordering is critical. If Core 0 updates lastUs after nowUs was sampled,
    // nowUs-lastUs wraps to about 2^32 us and looks like a 71-minute stall.
    // Reading the shared timestamp first guarantees it is not in our future.
    uint32_t lastUs = core1LastProgressUs;
    uint32_t nowUs = micros();
    uint32_t ageMs = (uint32_t)((nowUs - lastUs) / 1000UL);
    uint16_t stage = core1StageId;

    if (ageMs >= CORE1_STALL_WARN_MS) {
      unsigned long nowMs = millis();
      if (!core1StallReported ||
          nowMs - core1LastStallReportMillis >= CORE1_STAGE_WARN_INTERVAL_MS) {
        if (!core1StallReported) core1StallStartUs = lastUs;
        core1StallReported = true;
        core1LastStallReportMillis = nowMs;
        core1StallCount++;
      }

      // Never touch Serial, WiFi, Blynk, a queue or a mutex here: Core 0 may
      // be stalled while owning any of them. Persist a minimal marker and use
      // the reset primitive directly through the common helper.
      if (ageMs >= CORE0_HARD_STALL_RESET_MS) {
        Core0StallRecoveryRecord &r = core0StallRecoveryRecord;
        const uint32_t priorCount =
            (r.magic == CORE0_STALL_RTC_MAGIC) ? r.resetCount : 0;
        r.magic = CORE0_STALL_RTC_MAGIC;
        r.resetCount = priorCount + 1U;
        r.detectedAtMs = nowMs;
        r.progressAgeMs = ageMs;
        r.stageId = stage;
        r.reserved = 0;
        diagnosticResetRequest("CORE0_HARD_STALL");
      }
    } else if (core1StallReported) {
      core1StallReported = false;
    }
  }
}

// ─── PERFORMANCE DIAGNOSTICS — internal counters/timers ─────────────────────
// These values run continuously but are published to Blynk only every 10s.
// They are diagnostic-only and never trigger a reset or alter normal control.
// ─── ADDITIONAL I2C DIAGNOSTICS — internal state ────────────────────────────
// These values run continuously. V45-V50 are published only by the existing
// 10-second diagnostic pathway, so the diagnostic channel remains low-overhead.
// Device IDs: 0=none, 1=ENS160, 2=AHT21, 3=BMI160.
// Operation IDs: 0=none, 1=read, 2=measurement/command, 3=configuration.
volatile uint32_t perfI2cErrorCount = 0;

static inline void perfI2cError()
{
  perfI2cErrorCount++;
}


// ─── PERSISTENT RTC CRASH CHECKPOINT ─────────────────────────────────────────
// Two alternating RTC-SRAM records are used so an interrupted write cannot
// destroy the last known-good checkpoint. RTC_DATA_ATTR survives ESP32 software
// resets and watchdog resets; it is NOT persistent across power loss.
#define RTC_CHECKPOINT_MAGIC       0x43504B54UL  // "CPKT"
#define RTC_CHECKPOINT_VERSION     2
#define RTC_CHECKPOINT_INTERVAL_MS 1000UL

// Keep this type definition below the forward declaration at the top; this
// avoids Arduino auto-prototype ordering errors during compilation.
struct RtcCheckpoint {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t sequence;
  uint32_t uptimeMs;
  uint32_t ppsCount;
  uint32_t sensorHeartbeat;
  uint8_t  sensorState;
  uint8_t  loopState;
  uint8_t  loopAlive;
  uint8_t  ppsAlive;
  uint32_t ppsAgeMs;
  uint32_t lastI2cDevice;
  uint32_t lastI2cOperation;
  uint32_t i2cErrorCount;
  uint16_t loopStageId;
  uint16_t reserved2;
  uint32_t loopStageAgeMs;
  uint32_t crc;
  char firmwareVersion[16];  // v3.9.26: track firmware version in RTC checkpoint
};

// v3.9.6: use RTC_NOINIT_ATTR so the checkpoint records are left untouched by
// startup initialization. This is specifically intended for reset-persistent
// diagnostics. The existing two-slot + CRC scheme is retained unchanged.
RTC_NOINIT_ATTR RtcCheckpoint rtcCheckpointSlots[2];
uint8_t rtcCheckpointActiveSlot = 0;
uint32_t lastRtcCheckpointMillis = 0;

// Snapshot of the checkpoint belonging to the run BEFORE this boot.
// Kept separate from the live RTC slots so V51 continues to report the
// crashed/previous run after the current run starts writing new checkpoints.
RtcCheckpoint rtcPreviousRunCheckpoint = {};
bool rtcPreviousRunCheckpointValid = false;

static uint32_t rtcCheckpointCrc(const RtcCheckpoint& cp)
{
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&cp);
  size_t n = offsetof(RtcCheckpoint, crc);
  uint32_t crc = 0xFFFFFFFFUL;
  while (n--) {
    crc ^= *p++;
    for (uint8_t bit=0; bit<8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1U)));
  }
  return ~crc;
}

static bool rtcCheckpointValid(const RtcCheckpoint& cp)
{
  return cp.magic == RTC_CHECKPOINT_MAGIC &&
         cp.version == RTC_CHECKPOINT_VERSION &&
         cp.crc == rtcCheckpointCrc(cp);
}

// v3.9.6: detailed RTC validation diagnostics. These are diagnostic-only and
// do not alter checkpoint selection or normal application logic.
static const char* rtcSlotStatus(const RtcCheckpoint& cp)
{
  bool magicOk = (cp.magic == RTC_CHECKPOINT_MAGIC);
  bool versionOk = (cp.version == RTC_CHECKPOINT_VERSION);
  bool crcOk = (cp.crc == rtcCheckpointCrc(cp));

  if (magicOk && versionOk && crcOk) return "VALID";
  if (!magicOk && !versionOk && cp.magic == 0 && cp.version == 0) return "EMPTY";
  if (!magicOk) return "BAD MAGIC";
  if (!versionOk) return "BAD VERSION";
  if (!crcOk) return "BAD CRC";
  return "INVALID";
}

static void rtcPrintSlotDiagnostics()
{
  DBG_PRINTF("[RTC] slot0: %s seq=%lu magic=0x%08lX version=%u crc=0x%08lX\n",
                rtcSlotStatus(rtcCheckpointSlots[0]),
                (unsigned long)rtcCheckpointSlots[0].sequence,
                (unsigned long)rtcCheckpointSlots[0].magic,
                (unsigned)rtcCheckpointSlots[0].version,
                (unsigned long)rtcCheckpointSlots[0].crc);
  DBG_PRINTF("[RTC] slot1: %s seq=%lu magic=0x%08lX version=%u crc=0x%08lX\n",
                rtcSlotStatus(rtcCheckpointSlots[1]),
                (unsigned long)rtcCheckpointSlots[1].sequence,
                (unsigned long)rtcCheckpointSlots[1].magic,
                (unsigned)rtcCheckpointSlots[1].version,
                (unsigned long)rtcCheckpointSlots[1].crc);
}

static bool rtcCheckpointNewer(const RtcCheckpoint& a, const RtcCheckpoint& b)
{
  return (int32_t)(a.sequence - b.sequence) > 0;
}

static bool rtcGetPreviousCheckpoint(RtcCheckpoint& out)
{
  bool a = rtcCheckpointValid(rtcCheckpointSlots[0]);
  bool b = rtcCheckpointValid(rtcCheckpointSlots[1]);
  if (!a && !b) return false;
  if (a && (!b || rtcCheckpointNewer(rtcCheckpointSlots[0], rtcCheckpointSlots[1]))) {
    out = rtcCheckpointSlots[0];
    rtcCheckpointActiveSlot = 0;
  } else {
    out = rtcCheckpointSlots[1];
    rtcCheckpointActiveSlot = 1;
  }
  return true;
}

static const char* rtcSensorStateText(uint8_t state)
{
  switch (state) {
    case SENSOR_STATE_RUN: return "RUN";
    case SENSOR_STATE_I2C: return "I2C";
    default: return "BOOT";
  }
}

static const char* rtcLoopStateText(uint8_t state)
{
  switch (state) {
    case LOOP_STATE_RUN: return "RUN";
    case LOOP_STATE_WIFI: return "WIFI";
    case LOOP_STATE_BLYNK: return "BLYNK";
    case LOOP_STATE_HEALTH: return "HEALTH";
    default: return "BOOT";
  }
}

static const char* rtcI2cDeviceText(uint32_t device)
{
  switch (device) {
    case 1: return "ENS160";
    case 2: return "AHT21";
    case 3: return "BMI160";
    default: return "NONE";
  }
}

static const char* rtcI2cOperationText(uint32_t operation)
{
  switch (operation) {
    case 1: return "READ";
    case 2: return "MEAS/CMD";
    case 3: return "CONFIG";
    default: return "NONE";
  }
}

static void rtcReportPreviousCheckpoint()
{
  RtcCheckpoint cp;
  if (!rtcGetPreviousCheckpoint(cp)) {
    rtcPreviousRunCheckpointValid = false;
    DBG_PRINTLN("[RTC] No valid previous-run checkpoint (first boot or power loss)");
    // v3.9.6: show exactly why the two RTC slots were rejected. This is
    // diagnostic-only and does not modify either RTC slot.
    rtcPrintSlotDiagnostics();
    return;
  }

  // v3.9.6: identify which slot supplied the previous-run checkpoint.
  DBG_PRINTF("[RTC] Previous checkpoint selected: slot%u\n",
                (unsigned)rtcCheckpointActiveSlot);
  rtcPrintSlotDiagnostics();

  // Preserve this exact pre-boot snapshot for remote diagnostics on V51.
  rtcPreviousRunCheckpoint = cp;
  rtcPreviousRunCheckpointValid = true;
  DBG_PRINTLN("[RTC] Previous run checkpoint:");
  Serial.printf("      uptime:        %lum %lus %lums\n",
                (unsigned long)(cp.uptimeMs/60000UL),
                (unsigned long)((cp.uptimeMs/1000UL)%60UL),
                (unsigned long)(cp.uptimeMs%1000UL));
  Serial.printf("      PPS count:     %lu\n", (unsigned long)cp.ppsCount);
  Serial.printf("      sensor HB:     %lu (%s)\n",
                (unsigned long)cp.sensorHeartbeat, rtcSensorStateText(cp.sensorState));
  Serial.printf("      last I2C:      %s / %s\n",
                rtcI2cDeviceText(cp.lastI2cDevice),
                rtcI2cOperationText(cp.lastI2cOperation));
  Serial.printf("      I2C errors:    %lu\n", (unsigned long)cp.i2cErrorCount);
  Serial.printf("      Core1 stage:  %s (%ums)\n", core1StageText(cp.loopStageId), (unsigned)cp.loopStageAgeMs);
  Serial.printf("      loop alive:    %s (%s)\n",
                cp.loopAlive ? "YES":"NO", rtcLoopStateText(cp.loopState));
  Serial.printf("      PPS alive:     %s (age %lums)\n",
                cp.ppsAlive ? "YES":"NO", (unsigned long)cp.ppsAgeMs);
}

static void rtcFormatPreviousRun(char* out, size_t outSize)
{
  if (!out || outSize == 0) return;

  if (!rtcPreviousRunCheckpointValid) {
    snprintf(out, outSize, "RTC: no valid previous checkpoint");
    return;
  }

  const RtcCheckpoint& cp = rtcPreviousRunCheckpoint;
  unsigned long minutes = (unsigned long)(cp.uptimeMs / 60000UL);
  unsigned long seconds = (unsigned long)((cp.uptimeMs / 1000UL) % 60UL);
  unsigned long millisPart = (unsigned long)(cp.uptimeMs % 1000UL);

  snprintf(out, outSize,
           "RTC: FW %s, uptime: %lum %lus %lums, PPS count: %lu, sensor HB: %lu (%s), "
           "sensor state: %s, last I2C: %s / %s, I2C errors: %lu, "
           "loop alive: %s, loop state: %s, PPS alive: %s, PPS age: %lums",
           cp.firmwareVersion,
           minutes, seconds, millisPart,
           (unsigned long)cp.ppsCount,
           (unsigned long)cp.sensorHeartbeat,
           rtcSensorStateText(cp.sensorState),
           rtcSensorStateText(cp.sensorState),
           rtcI2cDeviceText(cp.lastI2cDevice),
           rtcI2cOperationText(cp.lastI2cOperation),
           (unsigned long)cp.i2cErrorCount,
           cp.loopAlive ? "YES" : "NO",
           rtcLoopStateText(cp.loopState),
           cp.ppsAlive ? "YES" : "NO",
           (unsigned long)cp.ppsAgeMs);
}

static void rtcCheckpointWrite()
{
  RtcCheckpoint cp = {};
  RtcCheckpoint previous;
  bool havePrevious = rtcGetPreviousCheckpoint(previous);
  cp.magic = RTC_CHECKPOINT_MAGIC;
  cp.version = RTC_CHECKPOINT_VERSION;
  cp.sequence = havePrevious ? previous.sequence+1UL : 1UL;
  cp.uptimeMs = millis();
  cp.ppsCount = core0SensorDiag.ppsCount;
  cp.sensorHeartbeat = core0SensorDiag.heartbeat;
  cp.sensorState = core0SensorDiag.sensorState;
  cp.loopState = loopState;
  cp.loopAlive = 1;
  cp.ppsAgeMs = core0SensorDiag.ppsAgeMs;
  cp.ppsAlive = (cp.ppsAgeMs <= 2000UL) ? 1 : 0;
  cp.lastI2cDevice = core0SensorDiag.lastI2cDevice;
  cp.lastI2cOperation = core0SensorDiag.lastI2cOperation;
  cp.i2cErrorCount = core0SensorDiag.i2cErrorCount;
  cp.loopStageId = core1StageId;
  cp.loopStageAgeMs = (uint32_t)((micros() - core1StageStartUs) / 1000UL);
  strncpy(cp.firmwareVersion, FIRMWARE_VERSION, sizeof(cp.firmwareVersion) - 1);
  cp.firmwareVersion[sizeof(cp.firmwareVersion) - 1] = '\0';
  cp.crc = rtcCheckpointCrc(cp);

  uint8_t nextSlot = havePrevious ? (rtcCheckpointActiveSlot ^ 1U) : 0;
  rtcCheckpointSlots[nextSlot] = cp;
  rtcCheckpointActiveSlot = nextSlot;

  // v3.9.6: verify the just-written slot immediately. This is diagnostic-only;
  // the existing checkpoint selection and alternating-slot logic are retained.
  bool writeOk = rtcCheckpointValid(rtcCheckpointSlots[nextSlot]);

  // v3.9.7: keep writing/verifying the RTC checkpoint every second, but do not
  // flood the Serial terminal with a message every second. Print the first
  // checkpoint immediately, then print the write/verify diagnostic once every
  // 10 minutes. This does not change the checkpoint frequency or stored data.
  static uint32_t lastRtcDiagnosticPrintMillis = 0;
  static bool rtcDiagnosticPrinted = false;
  const uint32_t RTC_DIAGNOSTIC_PRINT_INTERVAL_MS = 600000UL; // 10 minutes
  uint32_t diagnosticNow = millis();

  if (!rtcDiagnosticPrinted ||
      diagnosticNow - lastRtcDiagnosticPrintMillis >= RTC_DIAGNOSTIC_PRINT_INTERVAL_MS) {
    lastRtcDiagnosticPrintMillis = diagnosticNow;
    rtcDiagnosticPrinted = true;

    DBG_PRINTF("[RTC] checkpoint write: slot%u seq=%lu uptime=%lums result=%s\n",
                  (unsigned)nextSlot,
                  (unsigned long)cp.sequence,
                  (unsigned long)cp.uptimeMs,
                  writeOk ? "OK" : "VERIFY FAILED");
  }
}

static void rtcCheckpointService()
{
  uint32_t now = millis();
  if (lastRtcCheckpointMillis == 0 ||
      now-lastRtcCheckpointMillis >= RTC_CHECKPOINT_INTERVAL_MS) {
    lastRtcCheckpointMillis = now;
    rtcCheckpointWrite();
  }
}



// v3.9.84: per-stage timing diagnostics. These are observational only and
// help identify which Core-0 application section is responsible for the large
// aggregate sensor-cycle times seen in the field logs.
static uint32_t sensorStageMaxEnsMs = 0;
static uint32_t sensorStageMaxInmpMs = 0;
static uint32_t sensorStageMaxCoMs = 0;
static uint32_t sensorStageMaxGpsMs = 0;
static uint32_t sensorStageMaxInavMs = 0;
static uint32_t sensorStageMaxOledMs = 0;
static uint32_t sensorStageWarnCount = 0;

// v3.9.108 compile-order fix: Arduino auto-prototype generation can place
// perfReport() before the later RX/PONG forensic block. Declare the forensic
// state and helper here so perfReport() sees them regardless of prototype order.
static volatile unsigned long blynkStockSessionRxProxyStartMillis = 0;
static volatile uint32_t blynkStockRxProxy10sCount = 0;
static volatile uint32_t blynkStockRxProxy70sCount = 0;
static volatile uint32_t blynkStockHeartbeatTimeoutInferredCount = 0;
static volatile uint32_t blynkStockRunBlockCount = 0;
static volatile uint32_t blynkStockRunBlockOverlapCount = 0;
static volatile uint32_t blynkStockRunBlockCross10sCount = 0;
static volatile uint32_t blynkStockRunBlockCross70sCount = 0;
static volatile uint32_t blynkStockRunBlockMaxMs = 0;
static volatile uint64_t blynkStockRunBlockTotalMs = 0;
static volatile uint32_t blynkStockLastRunRxAgeBeforeMs = 0;
static volatile uint32_t blynkStockLastRunRxAgeAfterMs = 0;
static volatile bool blynkStockRxProxyActive = false;
static bool blynkStockRxProxy10sReported = false;
static bool blynkStockRxProxy70sReported = false;
static unsigned long blynkStockRxForensicLastLogMillis = 0;
static uint32_t blynkStockSessionNumber = 0;
static inline unsigned long blynkStockRxProxyAgeMs(unsigned long now = millis());

void perfReport()
{
  Serial.printf(
    "[PERF] PPS=%lu age=%lums loopMax=%lums sensorMax=%lums "
    "BlynkRunMax=%lums BlynkGuard=%lu lastGuard=%lums guardMax=%lums GPSmax=%lums I2Cmax=%lums FASTmax=%lums "
    "SLOWmax=%lums PPSledMax=%lums I2Cdev=%lu I2Cop=%lu "
    "I2Cerrors=%lu AHTmax=%lums ENSmax=%lums BMImax=%lums "
    "Core1Max=%lums Core1Stalls=%lu stageENS=%lums stageINMP=%lums stageCO=%lums stageGPS=%lums stageINAV=%lums stageOLED=%lums stageWarn=%lu\n",
    (unsigned long)core0SensorDiag.ppsCount,
    (unsigned long)core0SensorDiag.ppsAgeMs,
    (unsigned long)perfMaxLoopGapMs,
    (unsigned long)core0SensorDiag.maxSensorCycleMs,
    (unsigned long)perfMaxBlynkRunMs,
    (unsigned long)blynkRunGuardCount,
    (unsigned long)blynkRunLastGuardMs,
    (unsigned long)blynkRunMaxGuardMs,
    (unsigned long)core0SensorDiag.maxGpsFeedMs,
    (unsigned long)core0SensorDiag.maxI2cPathMs,
    (unsigned long)perfMaxFastSendMs,
    (unsigned long)perfMaxSlowSendMs,
    (unsigned long)core0SensorDiag.maxPpsLedResponseMs,
    (unsigned long)core0SensorDiag.lastI2cDevice,
    (unsigned long)core0SensorDiag.lastI2cOperation,
    (unsigned long)core0SensorDiag.i2cErrorCount,
    (unsigned long)core0SensorDiag.maxAht21Ms,
    (unsigned long)core0SensorDiag.maxEns160Ms,
    (unsigned long)core0SensorDiag.maxBmi160Ms,
    (unsigned long)core1MaxStageMs,
    (unsigned long)core1StallCount,
    (unsigned long)core0SensorDiag.stageMaxEnsMs,
    (unsigned long)core0SensorDiag.stageMaxInmpMs,
    (unsigned long)core0SensorDiag.stageMaxCoMs,
    (unsigned long)core0SensorDiag.stageMaxGpsMs,
    (unsigned long)core0SensorDiag.stageMaxInavMs,
    (unsigned long)core0SensorDiag.stageMaxOledMs,
    (unsigned long)core0SensorDiag.stageWarnCount
  );
  Serial.printf("[SERVER] tcpConnect=%lu tcpFail=%lu tcpTimeout=%lu connectLast=%lums probeStart=%lu probeOK=%lu probeFail=%lu probeMax=%lums handshakeMax=%lums blynkRunCalls=%lu runMax=%lums runSlow=%lu txCalls=%lu txMax=%lums writeEAGAIN=%lu writeTimeout=%lu socketErr=%lu rxAge=%lums connAge=%lums txQueue=%u lastErr=%ld soErr=%ld heap=%u minHeap=%u\n",
                 (unsigned long)blynkTransportConnectCalls, (unsigned long)blynkTransportConnectFailures,
                 (unsigned long)blynkTransportConnectTimeouts,
                 (unsigned long)blynkTransportConnectLastMs,
                (unsigned long)diagServerProbeStarts, (unsigned long)diagServerProbeSuccess,
                (unsigned long)diagServerProbeFails, (unsigned long)diagServerProbeMaxMs,
                (unsigned long)diagBlynkHandshakeMaxMs, (unsigned long)diagBlynkRunCalls,
                (unsigned long)perfMaxBlynkRunMs, (unsigned long)diagBlynkRunSlow,
                (unsigned long)blynkTxCallCount, (unsigned long)blynkTxMaxCallMs,
                (unsigned long)blynkTransportWriteEagain, (unsigned long)blynkTransportWriteTimeouts,
                (unsigned long)blynkTransportSocketErrors,
                 blynkStockRxProxyAgeMs(),
                (unsigned long)blynkTransportConnectionAgeMs, (unsigned)blynkTxPendingCount(),
                (long)blynkTransportLastErrno, (long)blynkTransportLastSoError,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  Serial.printf("[SERVER RX/PONG] protocolInboundAge=%lums cross10=%lu cross70=%lu inferredHeartbeatLoss=%lu runBlocks=%lu overlap=%lu runCross10=%lu runCross70=%lu blockMax=%lums blockTotal=%llums\n", blynkStockRxProxyAgeMs(), (unsigned long)blynkStockRxProxy10sCount, (unsigned long)blynkStockRxProxy70sCount, (unsigned long)blynkStockHeartbeatTimeoutInferredCount, (unsigned long)blynkStockRunBlockCount, (unsigned long)blynkStockRunBlockOverlapCount, (unsigned long)blynkStockRunBlockCross10sCount, (unsigned long)blynkStockRunBlockCross70sCount, (unsigned long)blynkStockRunBlockMaxMs, (unsigned long long)blynkStockRunBlockTotalMs);

}


#define SENSOR_TASK_HEALTH_TIMEOUT_MS  30000UL
#define HEALTH_REPORT_INTERVAL_MS      60000UL

// ─── INERTIAL NAV — EKF state (v2.3) ────────────────────────────────────────
float         inavX[4]    = {0,0,0,0};  // [E(m), N(m), v(m/s), heading(rad)]
float         inavP[4][4] = {{0}};      // covariance, initialised on first GPS fix
float         inavGyroBiasDps = 0.0f;   // gyro Z bias, calibrated via ZUPT
unsigned long zuptStartMillis = 0;
float         inavLastAlt = SENSOR_UNAVAILABLE;
bool          bmi160Ready = false;

// IMU variance-based stillness tracker (see INAV_IMU_* constants above).
// EMA state persists across calls; imuConfirmedStationary is the flag
// inertialNav() consults before trusting a nonzero GPS speed reading.
float         imuAccelMagEma         = 1.0f;   // EMA of accel magnitude (g)
float         imuAccelVarEma         = 0.0f;   // EMA of accel-magnitude variance (g^2)
float         imuGyroMagEma          = 0.0f;   // EMA of gyro magnitude (dps)
float         imuGyroVarEma          = 0.0f;   // EMA of gyro-magnitude variance (dps^2)
bool          imuVarEmaInited        = false;
unsigned long imuStillStartMillis    = 0;
bool          imuConfirmedStationary = false;

BlynkTimer blynkTimer;

// ============================================================================
// HELPERS
// ============================================================================

int averageAnalogRead(int pin)
{
  unsigned long sum = 0;
  for (byte i = 0; i < 9; i++) { sum += analogRead(pin); delayMicroseconds(100); }
  return (int)(sum / 9);
}

float adcToSensorVoltage(int counts)
{
  // Convert post-divider ADC counts to actual sensor voltage (multiplied by divider_scale)
  return counts * (3.3f / 4095.0f) * divider_scale;
}

// Convert raw ADC counts to actual voltage AT the ADC pin (no scale, just V)
float adcToVolts(int counts)
{
  return counts * (3.3f / 4095.0f);
}

uint8_t rssiToQuality(int32_t rssi)
{
  if (rssi >= -50)  return 100;
  if (rssi <= -100) return 0;
  return (uint8_t)(2 * (rssi + 100));
}

// Refresh the small Core0 -> Core1 connection-value queue at a bounded rate.
// This is data publication only: Core 1 never calls WiFi and never controls
// connection state. The sensor serializer merely includes the latest values
// in V62 when it seals the next pair.
static void core0PublishConnectionSnapshot(bool force)
{
  static unsigned long lastPublishMs = 0;
  const unsigned long now = millis();
  if (!force && now - lastPublishMs < 500UL) return;
  lastPublishMs = now;

  Core0ConnectionSnapshot connection = {};
  connection.wifiConnected = (WiFi.status() == WL_CONNECTED);
  connection.rssi = connection.wifiConnected ? WiFi.RSSI() : 0;
  connection.wifiQuality = connection.wifiConnected
                         ? rssiToQuality(connection.rssi) : 0;
  if (connectionSnapshotQueue != NULL) {
    xQueueOverwrite(connectionSnapshotQueue, &connection);
  }
}

void engMsg(const char* msg)
{
  if (xPortGetCoreID() == 1) {
    sensorLogPrintf("[ENG] %s\n", msg ? msg : "");
    // Core 1 owns sd. No cross-core mutex or wait is required here.
    strncpy(sd.eng_msg, msg ? msg : "", sizeof(sd.eng_msg)-1);
    sd.eng_msg[sizeof(sd.eng_msg)-1] = '\0';
    sd.eng_msg_at_ms = millis();
  } else {
    // Core 0 owns connection/application messages. They are overlaid on a
    // queued sensor snapshot only at publication time and never written to sd.
    Serial.print("[ENG] "); Serial.println(msg ? msg : "");
    strncpy(core0EngineeringMessage, msg ? msg : "", sizeof(core0EngineeringMessage)-1);
    core0EngineeringMessage[sizeof(core0EngineeringMessage)-1] = '\0';
    core0EngineeringMessageAtMs = millis();
  }
}

void engMsgf(const char* fmt, ...)
{
  char buf[128];
  va_list args; va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  engMsg(buf);
}

const char* resetReasonToString(esp_reset_reason_t reason)
{
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

void buzzerTone(uint16_t frequency, uint16_t durationMs)
{
  if (!frequency || !durationMs) return;
  if (xPortGetCoreID() == 0) {
    const BuzzerCommand command = {BUZZER_CMD_TONE, frequency, durationMs, 0};
    if (buzzerCommandQueue != NULL) xQueueSend(buzzerCommandQueue, &command, 0);
    return;
  }
  // Non-blocking: only schedule the tone. buzzerService() performs the actual
  // LEDC stop later, so no sensor/Blynk task is held up by a beep.
  buzzerPatternRunning = false;
  buzzerPatternCount = 0;
  buzzerPatternIndex = 0;
  // v3.9.4: the buzzer is permanently attached to its own LEDC channel/group.
  // Do NOT attach/detach it for every tone: the previous implementation entered
  // ledcAttachChannel() immediately after WiFi came up and still reproduced the
  // CPU1 Interrupt-WDT. Changing only the buzzer's dedicated timer is much safer.
  if (!buzzerLEDCReady) return;
  ledcChangeFrequencySafe(buzzPin, frequency, 8);
  ledcWriteSafe(buzzPin, 128);
  buzzerActiveFrequency = frequency;
  buzzerActive = true;
  buzzerStopMillis = millis() + durationMs;
}

void buzzerPattern3(uint16_t frequency, uint16_t durationMs, uint16_t pauseMs)
{
  if (!frequency || !durationMs) return;
  if (xPortGetCoreID() == 0) {
    const BuzzerCommand command = {BUZZER_CMD_PATTERN3, frequency, durationMs, pauseMs};
    if (buzzerCommandQueue != NULL) xQueueSend(buzzerCommandQueue, &command, 0);
    return;
  }
  buzzerPattern[0] = {frequency, durationMs, pauseMs};
  buzzerPattern[1] = {frequency, durationMs, pauseMs};
  buzzerPattern[2] = {frequency, durationMs, 0};
  buzzerPatternCount = 3;
  buzzerPatternIndex = 0;
  buzzerPatternRunning = true;
  buzzerActive = false;
  buzzerPatternNextMillis = millis();
}

void buzzerService()
{
  unsigned long now = millis();

  if (buzzerActive) {
    if ((long)(now - buzzerStopMillis) >= 0) {
      ledcWriteSafe(buzzPin, 0);
      // v3.9.4: keep the buzzer LEDC channel attached permanently. Detaching and
      // re-attaching it repeatedly is unnecessary and exercises the LEDC allocator
      // on Core 1 while the CO heater is active on Core 0.
      buzzerActive = false;
      if (buzzerPatternRunning) {
        buzzerPatternNextMillis = now + buzzerPattern[buzzerPatternIndex].pauseMs;
      }
    }
    return;
  }

  if (buzzerPatternRunning && (long)(now - buzzerPatternNextMillis) >= 0) {
    BuzzerPatternStep step = buzzerPattern[buzzerPatternIndex];
    // v3.9.4: use the buzzer's permanently attached, dedicated LEDC timer.
    if (!buzzerLEDCReady) {
      buzzerPatternRunning = false;
      return;
    }
    ledcChangeFrequencySafe(buzzPin, step.frequency, 8);
    ledcWriteSafe(buzzPin, 128);
    buzzerActiveFrequency = step.frequency;
    buzzerActive = true;
    buzzerStopMillis = now + step.durationMs;
    buzzerPatternIndex++;
    if (buzzerPatternIndex >= buzzerPatternCount) {
      buzzerPatternRunning = false;
    }
  }
}

// ─── NON-BLOCKING BLYNK TX QUEUE ──────────────────────────────────────────────
// Blynk v0.6.1 applies BLYNK_MSG_LIMIT inside sendCmd().  A burst of many
// virtualWrite() calls therefore makes sendCmd() call Blynk.run() in a loop
// while waiting for the next permitted message slot.  FAST + SLOW + the
// diagnostics callback can coincide and create multi-second loop stalls.
// Those stalls are visible in the log as FASTmax/loopMax spikes (up to ~11s)
// and can starve the Blynk heartbeat/recovery state machine.
//
// Fix: safeWrite*() now only updates a coalescing queue.  Core 1 transmits
// at most one queued virtual-write every BLYNK_TX_INTERVAL_MS.  If the same
// V-pin is queued again before transmission, its value is replaced rather than
// adding another stale packet.  Blynk.run() therefore gets a chance to run
// between every outgoing packet and telemetry bursts can never monopolise
// loop().
//
// This is intentionally additive: the existing V-pin list and update
// intervals remain unchanged.
#define BLYNK_TX_QUEUE_SIZE      64
#define BLYNK_TX_VALUE_LEN       256

// v3.9.91: the queue is a coalescing backlog, not an unlimited send buffer.
// With the stock ESP32 WiFiClient path, one virtualWrite() can still block for
// several seconds when the peer/socket stops accepting bytes. During that
// event, continuing to enqueue every dashboard/diagnostic channel only creates
// a larger backlog and makes recovery slower. V61 is the authoritative
// coherent record; the individual dashboard pins are therefore deliberately
// treated as lower-priority convenience telemetry.
#define BLYNK_TX_SOFT_LIMIT       8U
#define BLYNK_TX_HARD_LIMIT       16U
uint32_t blynkTxCongestionSkips = 0;
uint32_t blynkTxHardSkips = 0;

// v3.9.43: slowed from 100ms (<=10 pkt/s) to 150ms (<=~6.7 pkt/s). Investigated
// alongside the Blynk server's own GC log: the server's occasional brief
// socket-read stalls correlate with G1 allocation/marking pressure, not with
// this device's total data volume (already ruled out — see the diagnostics-
// cadence change above, which cut volume 3x with no change in stall
// frequency). What IS still adjustable here is burstiness: a smoother,
// slightly slower packet cadence is less likely to land a write inside
// whatever brief window the server's socket-read loop is unavailable in,
// even though it cannot prevent that window from existing. This does not
// reduce total information sent — the coalescing queue (see block comment
// above) already collapses repeated same-pin updates, so a slightly longer
// interval mainly spreads out delivery, it does not drop data.
#define BLYNK_TX_INTERVAL_MS     500UL   // <=2 writes/s inside the bounded send window
#define BLYNK_WEAK_LINK_TX_INTERVAL_MS 1000UL
#define BLYNK_WEAK_LINK_ENTER_RSSI_DBM (-67)
#define BLYNK_WEAK_LINK_EXIT_RSSI_DBM  (-63)
static bool blynkWeakLinkMode = false;
static uint32_t blynkWeakLinkEntries = 0;
// v3.9.93: do not generate 31 dashboard writes as a burst.  One legacy
// dashboard V-pin is refreshed per second in round-robin order.  This keeps
// every V1-V32 dashboard channel alive while keeping the socket workload smooth.
#define BLYNK_DASHBOARD_ROUNDROBIN_MS 4000UL
// Diagnostics are also spread out instead of publishing V34-V65 as one burst.
#define BLYNK_DIAG_ROUNDROBIN_MS      15000UL
#define BLYNK_COHERENT_RECORD_MS      1000UL
#define BLYNK_V4_INTERVAL_MS          1000UL
// A stock NetworkClient write can still return slowly. Pause application
// telemetry after such a write, but do not destroy a still-connected session;
// Blynk.run() remains responsible for deciding whether the session is alive.
#define BLYNK_STOCK_TX_STALL_MS       750UL
// v3.9.117: independent non-blocking TCP reachability gate for application TX.
// This does NOT replace the stock Blynk transport. It opens a separate short-lived
// TCP connection to the configured Blynk endpoint and is used only to decide
// whether it is sensible to enter stock WiFiClient::write() for telemetry.
#define BLYNK_STOCK_TX_PROBE_INTERVAL_MS 5000UL
#define BLYNK_STOCK_TX_PROBE_MAX_AGE_MS  7000UL
#define BLYNK_STOCK_TX_PROBE_FAILS_BEFORE_REFRESH 2U
#define BLYNK_STOCK_TX_PROBE_TIMEOUT_MS  350UL
#define BLYNK_STOCK_TX_PING_INTERVAL_MS  10000UL
#define BLYNK_STOCK_TX_PING_SETTLE_MS    300UL
// These declarations are needed by blynkTxService(), which appears earlier
// in the file than the main Blynk state section. Keep them here so the C++
// compiler sees both the macro and variables before that function body.
// v3.9.118: prevent a dead/half-working Blynk endpoint from turning the
// Core-1 loop into an endless 3s-connect / 5s-retry churn.  A successful TCP
// connect is not proof that the Blynk protocol service is accepting logins.
// When Blynk.run() repeatedly fails the login, back off progressively and
// clean-refresh the protocol object only after several failures. The local-LAN
// retry starts at 1s and remains bounded; WiFi remains intact.
#define BLYNK_STOCK_RETRY_INTERVAL_MS          1000UL
#define BLYNK_STOCK_RETRY_MAX_MS               8000UL
#define BLYNK_STOCK_REFRESH_HOLDOFF_MS         5000UL
#define BLYNK_STOCK_LONG_HOLDOFF_MS           30000UL
#define BLYNK_STOCK_REFRESHES_BEFORE_LONG_HOLDOFF 3U
// v3.9.212: boot-only grace after the first GOT_IP. The previous immediate
// attempt repeatedly hit a full 1s TCP timeout, while the next attempt worked.
// True WiFi re-association/recovery does not use this delay.
#define BLYNK_FIRST_ASSOCIATION_GRACE_MS        3000UL
static unsigned long blynkStockNextRunMillis = 0;
unsigned long blynkOfflineSince = 0;

struct BlynkTxItem {
  bool pending;
  uint8_t vpin;
  char value[BLYNK_TX_VALUE_LEN];
};

BlynkTxItem blynkTxQueue[BLYNK_TX_QUEUE_SIZE] = {};
uint8_t blynkTxNext = 0;
unsigned long blynkTxLastSend = 0;
uint32_t blynkTxDropped = 0;

// v3.9.107 FORENSIC V61 test state. These declarations must appear before
// blynkTxService(), which records V61 transmit timing and sequence data.
static uint8_t v61TestValue = 1;
static unsigned long v61TestLastEndMillis = 0;
static unsigned long v61TestLastStartMillis = 0;

// V61/V62 coherent-pair lock. The generic queue coalesces by V-pin, which is
// correct for ordinary dashboard telemetry but unsafe for the two halves of a
// database/GIS record. Keep one pair immutable until both halves are delivered
// or the session is abandoned.
static bool blynkCoherentPairActive = false;
static bool blynkCoherentPairSecondDue = false;
static unsigned long blynkCoherentPairDeviceMs = 0;
static long blynkCoherentPairGpsTs = 0;
static uint32_t blynkCoherentPairSequence = 0;
static uint32_t blynkLastAcceptedTelemetrySequence = 0;


// v3.9.20: instrument the Blynk transport itself. This is
// deliberately below Blynk.virtualWrite() so we can distinguish a stall in
// BlynkProtocol::sendCmd()/BLYNK_MSG_LIMIT from a stall in WiFiClient::write().
volatile uint32_t blynkTransportWriteCalls = 0;
volatile uint32_t blynkTransportWriteZero = 0;
volatile uint32_t blynkTransportWriteMaxMs = 0;
volatile uint32_t blynkTransportWriteLastMs = 0;
volatile uint32_t blynkTransportReadCalls = 0;
volatile uint32_t blynkTransportReadMaxMs = 0;
volatile uint32_t blynkTransportReadLastMs = 0;

// v3.9.22 deep network diagnostics. These counters survive for the entire boot
// and are included in the periodic diagnostic snapshot.
volatile uint32_t blynkTransportConnectCalls = 0;
volatile uint32_t blynkTransportConnectFailures = 0;
volatile uint32_t blynkTransportConnectTimeouts = 0;
volatile uint32_t blynkTransportConnectLastMs = 0;
// v3.9.40: last time the raw transport actually received real bytes from the
// peer (see BoundedBlynkArduinoClient::read() below). This is the only true
// proof-of-life signal for the TCP session. Blynk.connected() and even our own
// connected() override can both keep reporting "connected" indefinitely on a
// half-open socket (peer process died/hung without sending FIN/RST — MSG_PEEK
// then just returns EAGAIN forever, see connected() comments). Without this,
// a stale session was only ever cleared by an unrelated WiFi-level disconnect
// event, which is why field failures took minutes to self-heal instead of
// recovering on their own shortly after the peer actually went silent.
volatile unsigned long blynkLastRxMillis = 0;

// v3.9.108 STOCK-BLYNK RX/PONG FORENSICS. The helper name is retained for
// compatibility with existing diagnostics. In CONNECTION_MODE 1, v3.9.185 now
// returns the patched protocol's authoritative last inbound activity age.
static inline unsigned long blynkStockRxProxyAgeMs(unsigned long now)
{
#if CONNECTION_MODE == 1
  // v3.9.185: the locally patched protocol exports its real inbound-activity
  // timestamp. The former value was session age and therefore increased even
  // while a RESPONSE arrived every ten seconds. Keep the helper name to avoid a
  // broad mechanical rewrite, but make its value authoritative.
  const unsigned long lastInbound = Blynk.lastInboundActivity();
  return lastInbound ? (unsigned long)(now - lastInbound) : 0UL;
#else
  return (blynkStockRxProxyActive && blynkStockSessionRxProxyStartMillis)
           ? (unsigned long)(now - blynkStockSessionRxProxyStartMillis) : 0UL;
#endif
}

// v3.9.112: these guard flags are referenced by the RX-forensic session
// helpers above the main TX-service declarations, so keep their storage
// declarations before those function bodies for Arduino auto-prototyping.
static bool blynkStockRxTxGuardActive = false;
static bool blynkStockRxTxWarnReported = false;
static bool blynkStockRxTxStaleReported = false;

// v3.9.117 stock-session TX admission state. The probe is deliberately separate
// from Blynk's own WiFiClient socket, so the Blynk transport remains stock.
static unsigned long blynkStockLastTxProbeOkMillis = 0;
static unsigned long blynkStockNextTxProbeMillis = 0;
static uint8_t blynkStockTxProbeFailures = 0;
static bool blynkStockTxProbeInFlight = false;
static unsigned long blynkStockLastPingMillis = 0;
static unsigned long blynkStockTxPingQuietUntil = 0;

static void blynkStockRxForensicSessionStart(unsigned long now)
{
  // A PING watch belongs to one TCP/Blynk session only. Never carry an armed
  // message ID across a reconnect and later classify it as a loss in the new
  // session.
  blynkPingRttWatchArmed = false;
  blynkStockSessionRxProxyStartMillis = now;
  blynkStockRxProxyActive = true;
  blynkStockRxProxy10sReported = false;
  blynkStockRxProxy70sReported = false;
  blynkStockRxForensicLastLogMillis = 0;
  // v3.9.112: seed application-TX freshness from the session handshake.
  // The passive header probe observed the stock RESPONSE that established the
  // session immediately before this function is called.
  forensicRspLastNewMillis = now;
  blynkStockRxTxGuardActive = false;
  blynkStockRxTxWarnReported = false;
  blynkStockRxTxStaleReported = false;
  ++blynkStockSessionNumber;
  Serial.printf("[Blynk RX OBSERVATION] SESSION #%lu START protocolInboundAge=0ms — authoritative BlynkProtocol inbound clock active\n", (unsigned long)blynkStockSessionNumber);
}

static void blynkStockRxForensicSessionEnd(unsigned long now, const char* reason)
{
  const unsigned long age = blynkStockRxProxyAgeMs(now);
  if (blynkStockRxProxyActive && age >= 50000UL) {
    ++blynkStockHeartbeatTimeoutInferredCount;
    Serial.printf("[Blynk RX OBSERVATION] SESSION #%lu END reason=%s protocolInboundAge=%lums RSSI=%ddBm wifiDiscEvents=%lu — verified inbound-silence pattern\n", (unsigned long)blynkStockSessionNumber, reason ? reason : "UNKNOWN", age, (int)WiFi.RSSI(), (unsigned long)wifiDisconnectEventSequence);
  } else {
    Serial.printf("[Blynk RX OBSERVATION] SESSION #%lu END reason=%s protocolInboundAge=%lums RSSI=%ddBm wifiDiscEvents=%lu\n", (unsigned long)blynkStockSessionNumber, reason ? reason : "UNKNOWN", age, (int)WiFi.RSSI(), (unsigned long)wifiDisconnectEventSequence);
  }
  blynkStockRxProxyActive = false;
  blynkStockSessionRxProxyStartMillis = 0;
  blynkStockRxProxy10sReported = false;
  blynkStockRxProxy70sReported = false;
}

static void blynkStockRxForensicService(const char* phase)
{
  const unsigned long now = millis();
  if (!blynkStockRxProxyActive || !Blynk.connected()) return;
  const unsigned long age = blynkStockRxProxyAgeMs(now);
  if (!blynkStockRxProxy10sReported && age >= 10000UL) {
    blynkStockRxProxy10sReported = true; ++blynkStockRxProxy10sCount;
    Serial.printf("[Blynk RX OBSERVATION] protocol inbound silence >=10s age=%lums phase=%s runMax=%lums blocks=%lu overlap=%lu\n", age, phase ? phase : "UNKNOWN", (unsigned long)perfMaxBlynkRunMs, (unsigned long)blynkStockRunBlockCount, (unsigned long)blynkStockRunBlockOverlapCount);
  }
  if (!blynkStockRxProxy70sReported && age >= 70000UL) {
    blynkStockRxProxy70sReported = true; ++blynkStockRxProxy70sCount;
    Serial.printf("[Blynk RX OBSERVATION] protocol inbound silence >=70s age=%lums phase=%s runMax=%lums blocks=%lu overlap=%lu\n", age, phase ? phase : "UNKNOWN", (unsigned long)perfMaxBlynkRunMs, (unsigned long)blynkStockRunBlockCount, (unsigned long)blynkStockRunBlockOverlapCount);
  }
  if (blynkStockRxForensicLastLogMillis == 0 || now - blynkStockRxForensicLastLogMillis >= 5000UL) {
    blynkStockRxForensicLastLogMillis = now;
    // v3.9.126 DIAGNOSTIC: the 3002ms Blynk.run() stall is fixed (Serial TX
    // buffer). What remains is periodic heartbeat-timeout disconnects —
    // V61 TX keeps succeeding right up to each death (no writeTimeout/
    // socketErr in [SERVER]), only inbound traffic goes quiet, and
    // WiFi.status() stays WL_CONNECTED throughout (no
    // ARDUINO_EVENT_WIFI_STA_DISCONNECTED fires around these deaths per
    // wifiDisconnectEventSequence). That pattern — uplink fine, downlink
    // silent, no full WiFi drop — is consistent with marginal RSSI/link
    // quality (asymmetric loss) or something server-side rather than a
    // WiFi association failure. Logging RSSI and the WiFi disconnect-event
    // counter on every periodic tick gives a trend line into each death
    // instead of only a single reading after the fact.
    Serial.printf("[Blynk RX OBSERVATION] age=%lums phase=%s WiFi=%d Blynk=%d runMax=%lums runSlow=%lu blocks=%lu overlap=%lu cross10=%lu cross70=%lu blockMax=%lums blockTotal=%llums lastRunBefore=%lums lastRunAfter=%lums txCalls=%lu txMax=%lums heap=%u RSSI=%ddBm wifiDiscEvents=%lu wifiLastDiscReason=%u\n", age, phase ? phase : "CONNECTED", (int)WiFi.status(), Blynk.connected() ? 1 : 0, (unsigned long)perfMaxBlynkRunMs, (unsigned long)diagBlynkRunSlow, (unsigned long)blynkStockRunBlockCount, (unsigned long)blynkStockRunBlockOverlapCount, (unsigned long)blynkStockRunBlockCross10sCount, (unsigned long)blynkStockRunBlockCross70sCount, (unsigned long)blynkStockRunBlockMaxMs, (unsigned long long)blynkStockRunBlockTotalMs, (unsigned long)blynkStockLastRunRxAgeBeforeMs, (unsigned long)blynkStockLastRunRxAgeAfterMs, (unsigned long)blynkTxCallCount, (unsigned long)blynkTxMaxCallMs, (unsigned)ESP.getFreeHeap(), (int)WiFi.RSSI(), (unsigned long)wifiDisconnectEventSequence, (unsigned)wifiLastDisconnectReason);
  }
}

static void blynkStockRecordRunForensic(uint32_t elapsedMs, unsigned long rxAgeBefore, const char* phase)
{
  const unsigned long now = millis();
  const unsigned long rxAgeAfter = blynkStockRxProxyAgeMs(now);
  blynkStockLastRunRxAgeBeforeMs = (uint32_t)rxAgeBefore;
  blynkStockLastRunRxAgeAfterMs = (uint32_t)rxAgeAfter;
  if (elapsedMs < 500UL) return;
  ++blynkStockRunBlockCount; blynkStockRunBlockTotalMs += elapsedMs;
  if (elapsedMs > blynkStockRunBlockMaxMs) blynkStockRunBlockMaxMs = elapsedMs;
  const bool staleBefore = rxAgeBefore >= 10000UL;
  const bool staleAfter = rxAgeAfter >= 10000UL;
  const bool cross10 = rxAgeBefore < 10000UL && rxAgeAfter >= 10000UL;
  const bool cross70 = rxAgeBefore < 70000UL && rxAgeAfter >= 70000UL;
  if (staleBefore || staleAfter) ++blynkStockRunBlockOverlapCount;
  if (cross10) ++blynkStockRunBlockCross10sCount;
  if (cross70) ++blynkStockRunBlockCross70sCount;
  Serial.printf("[Blynk RUN BLOCK FORENSIC] phase=%s elapsed=%lums observedResponseAgeBefore=%lums observedResponseAgeAfter=%lums staleOverlap=%d cross10=%d cross70=%d blocks=%lu maxBlock=%lums — long stock Blynk.run() can delay protocol processing inside the blocking call\n", phase ? phase : "UNKNOWN", (unsigned long)elapsedMs, (unsigned long)rxAgeBefore, (unsigned long)rxAgeAfter, (staleBefore || staleAfter) ? 1 : 0, cross10 ? 1 : 0, cross70 ? 1 : 0, (unsigned long)blynkStockRunBlockCount, (unsigned long)blynkStockRunBlockMaxMs);
}
volatile uint32_t blynkTransportWriteEagain = 0;
volatile uint32_t blynkTransportWriteTimeouts = 0;
volatile uint32_t blynkTransportSocketErrors = 0;
volatile int32_t  blynkTransportLastErrno = 0;
volatile int32_t  blynkTransportLastSoError = 0;
volatile int32_t  blynkTransportLastSndBuf = 0;
volatile uint16_t blynkTransportLastLocalPort = 0;
volatile uint16_t blynkTransportLastRemotePort = 0;
volatile uint32_t blynkTransportConnectionAgeMs = 0;
volatile uint32_t blynkTransportLastFailureAgeMs = 0;
volatile bool     blynkTransportKeepaliveConfigured = false;

// Map has a different Blynk command shape (index, lat, lon, label), so keep
// one latest pending map packet alongside the normal V-pin queue.
bool blynkMapPending = false;
long blynkMapIndex = 0;
double blynkMapLat = 0.0;
double blynkMapLng = 0.0;

static bool blynkTxQueueValue(uint8_t vpin, const char* value)
{
  if (!value) value = "";

  // Coalesce an already queued V-pin.  This keeps the queue bounded and,
  // importantly, means a reconnect sends the newest sensor value.
  for (uint8_t i = 0; i < BLYNK_TX_QUEUE_SIZE; ++i) {
    if (blynkTxQueue[i].pending && blynkTxQueue[i].vpin == vpin) {
      strncpy(blynkTxQueue[i].value, value, BLYNK_TX_VALUE_LEN - 1);
      blynkTxQueue[i].value[BLYNK_TX_VALUE_LEN - 1] = '\0';
      return true;
    }
  }

  for (uint8_t n = 0; n < BLYNK_TX_QUEUE_SIZE; ++n) {
    uint8_t i = (uint8_t)((blynkTxNext + n) % BLYNK_TX_QUEUE_SIZE);
    if (!blynkTxQueue[i].pending) {
      blynkTxQueue[i].pending = true;
      blynkTxQueue[i].vpin = vpin;
      strncpy(blynkTxQueue[i].value, value, BLYNK_TX_VALUE_LEN - 1);
      blynkTxQueue[i].value[BLYNK_TX_VALUE_LEN - 1] = '\0';
      return true;
    }
  }

  // This should only happen if all 64 different V-pins are simultaneously
  // pending.  Dropping the oldest queued item is preferable to blocking the
  // main loop trying to make room.
  blynkTxQueue[blynkTxNext].pending = false;
  blynkTxDropped++;
  blynkTxQueue[blynkTxNext].pending = true;
  blynkTxQueue[blynkTxNext].vpin = vpin;
  strncpy(blynkTxQueue[blynkTxNext].value, value, BLYNK_TX_VALUE_LEN - 1);
  blynkTxQueue[blynkTxNext].value[BLYNK_TX_VALUE_LEN - 1] = '\0';
  return true;
}

void safeWrite(int vpin, double val)
{
  char value[BLYNK_TX_VALUE_LEN];
  snprintf(value, sizeof(value), "%.6f", val);
  blynkTxQueueValue((uint8_t)vpin, value);
}

void safeWriteI(int vpin, int val)
{
  char value[24];
  snprintf(value, sizeof(value), "%d", val);
  blynkTxQueueValue((uint8_t)vpin, value);
}

void safeWriteS(int vpin, const char* val)
{
  blynkTxQueueValue((uint8_t)vpin, val);
}

// v3.9.23: TX health timestamps must be declared before blynkTxService().
// Arduino's auto-generated function prototypes can otherwise see the service
// function before the later global declarations.
unsigned long blynkLastTxAttemptMillis = 0;
unsigned long blynkLastTxSuccessMillis = 0;
// v3.9.43: brief pause on the TX (telemetry-sending) side only, right after a
// FRESH Blynk connection is established. Does not affect Blynk.run(),
// heartbeat pings, or the handshake itself — only blynkTxService() checks
// this. Rationale: a reconnect after an outage often has a backed-up queue
// (observed txPending=46 in a field log right after a write-timeout
// recovery); bursting through that backlog at full TX pace immediately after
// the server has just re-accepted the connection is the worst possible
// moment to hit it hard again if the underlying cause was brief server-side
// pressure. This grace period does not fix the server-side cause (see the
// companion G1GC tuning), it only avoids compounding it from this device's
// side, and it costs at most BLYNK_TX_POST_RECONNECT_GRACE_MS of telemetry
// latency once per reconnect — negligible next to the outage itself.
#define BLYNK_TX_POST_RECONNECT_GRACE_MS 2000UL
unsigned long blynkTxGraceUntil = 0;
// v3.9.101: application-TX circuit breaker.  Blynk.run() continues during
// quarantine; only virtualWrite/timer traffic is suppressed.
// v3.9.147: 3s is enough to prevent an immediate burst retry after a slow
// write while Blynk.run() continues servicing the same live session. The old
// 8s pause delayed otherwise healthy telemetry without improving liveness.
#define BLYNK_STOCK_TX_QUARANTINE_MS 3000UL
#define BLYNK_STOCK_POST_RECONNECT_STABLE_MS 2000UL
// v3.9.180/v3.9.185: stop application traffic at the first unanswered
// 10-second protocol interval. Blynk.run(), protocol PING and TCP
// retransmission remain active; only application frames are suppressed.
#define BLYNK_STOCK_RX_TX_GUARD_MS 45000UL
#define BLYNK_STOCK_RX_TX_WARN_MS 30000UL
#define BLYNK_STOCK_RX_TX_STALE_MS 60000UL
#define BLYNK_STOCK_PRE_PING_QUIET_RX_MS 6000UL
#define BLYNK_STOCK_APPLICATION_TX_PAUSE_RX_MS 10000UL
static unsigned long blynkStockTxQuarantineUntil = 0;
static unsigned long blynkStockStableUntilMillis = 0;
static uint8_t blynkDashboardRoundRobinIndex = 0;
static uint8_t blynkDiagRoundRobinIndex = 0;
static unsigned long blynkDashboardNextMillis = 0;
static unsigned long blynkDiagNextMillis = 0;
static bool blynkStockTxRecoveryPending = false;
static bool blynkStockRxTrafficBrakeActive = false;

// Stock Blynk connection/session globals must be declared before the
// supervisor functions below; Arduino's prototype generation does not
// guarantee that later globals are visible to those function bodies.
bool blynkConfigured = false;
unsigned long lastBlynkForcePing = 0;
uint32_t blynkForcePingCount = 0;
unsigned long blynkLastForcePingMillis = 0;
unsigned long lastBlynkConnectAttempt = 0;
unsigned long blynkLastConnectedMillis = 0;
bool blynkHandshakeActive = false;
unsigned long blynkHandshakeStarted = 0;
uint32_t blynkHandshakeWiFiDisconnectSeq = 0;
#define BLYNK_OFFLINE_RECOVERY_MS 60000UL
#define BLYNK_HANDSHAKE_FAILS_BEFORE_WIFI_RESET 2U
#define BLYNK_WIFI_RECOVERY_CYCLE_THRESHOLD 2U
static uint8_t blynkHandshakeFailCount = 0;
static uint8_t blynkWifiRecoveryFailCount = 0;
unsigned long blynkStartupHoldoffUntil = 0;
static bool blynkConnectImmediate = false;
static bool blynkForceReconnect = false;

// v3.9.94: independent stock-Blynk session supervisor.  Blynk.connected()
// is an input to this state machine, not its sole source of truth.  Recovery
// is deliberately short: once a dead/stalled session is detected, close the
// Blynk socket, wait only a brief settle interval, then let stock Blynk.run()
// create a fresh session.  No WiFi teardown is involved.
enum BlynkStockSessionState : uint8_t {
  BLYNK_STOCK_SESSION_IDLE = 0,
  BLYNK_STOCK_SESSION_CONNECTING,
  BLYNK_STOCK_SESSION_ONLINE,
  BLYNK_STOCK_SESSION_RECOVERY
};
static BlynkStockSessionState blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
static unsigned long blynkStockStateSinceMillis = 0;
static uint32_t blynkStockReconnectCount = 0;
static uint32_t blynkStockConsecutiveRetryFailures = 0;
static uint8_t blynkStockCleanRefreshCount = 0;
static unsigned long blynkStockLastCleanRefreshMillis = 0;
#define BLYNK_STOCK_CLEAN_REFRESH_AFTER_RETRIES 3U
#define BLYNK_STOCK_CLEAN_REFRESH_MIN_MS 10000UL
// v3.9.148: WL_CONNECTED only proves association; it does not prove that the
// ESP32 lwIP/netif can create a usable new TCP stream after a poisoned send
// queue. Permit one radio/netif rebuild per Blynk outage, only after normal
// Blynk-only recovery has already failed.
// v3.9.171: the v3.9.170 field log proved that a soft STA disconnect/reconnect
// can leave this failed state intact (27 association events / 102 seconds on
// the first recovery, then no recovery until an MCU reset). The escalation now
// stops and restarts the ESP32 WiFi driver while retaining credentials. It is
// still attempted only once per Blynk outage and does not affect ESP8266 code.
#define BLYNK_STOCK_WIFI_STACK_RECOVERY_MS 20000UL
#define BLYNK_STOCK_WIFI_DRIVER_SETTLE_MS 2000UL
// v3.9.157: a local-LAN TCP SYN/SYN-ACK normally completes in milliseconds.
// Two consecutive full-duration 1s timeouts after a real online session are enough
// to distinguish the measured dead downlink from a single lost packet, while
// excluding immediate TCP refusals and recovering before exponential holdoffs.
#define BLYNK_STOCK_RX_PATH_TIMEOUTS_BEFORE_RESTART 2U
static bool blynkStockWifiStackRecoveryAttempted = false;
static bool blynkStockWifiStackRecoveryInProgress = false;
static uint32_t blynkStockWifiStackRecoveryCount = 0;
enum SharedPathRecoveryState : uint8_t {
  SHARED_PATH_IDLE = 0,
  SHARED_PATH_QUIESCE,
  SHARED_PATH_WAIT_GOT_IP,
  SHARED_PATH_POST_IP_GRACE
};
static SharedPathRecoveryState sharedPathRecoveryState = SHARED_PATH_IDLE;
static uint32_t sharedPathRecoveryStateAtMs = 0;
static bool sharedPathForceFullScan = false;
#define SHARED_PATH_QUIESCE_MS 1250UL
#define SHARED_PATH_DRIVER_SETTLE_MS 3000UL
#define SHARED_PATH_POST_IP_GRACE_MS 5000UL
// Counts ACTUAL failed NetworkClient::connect() calls across clean Blynk
// protocol refreshes. v3.9.170 incorrectly incremented this once per failed
// Blynk.run() pass, including passes which did not issue a TCP connection.
static uint8_t blynkStockOutageTcpConnectFailures = 0;
static uint32_t blynkStockOutageConnectFailureBaseline = 0;
static uint8_t blynkStockOutageTcpConnectTimeouts = 0;
static uint32_t blynkStockOutageConnectTimeoutBaseline = 0;
static bool blynkStockTcpPreflightReady = false;
static uint8_t blynkStockTcpPreflightFailures = 0;
#define BLYNK_STOCK_TCP_PREFLIGHT_TIMEOUT_MS 1000UL
static uint32_t blynkStockRecoveryCount = 0;
static uint32_t blynkStockLastRecoveryReason = 0;
static unsigned long blynkStockRecoveryStartedMillis = 0;
#define BLYNK_STOCK_RECOVERY_SETTLE_MS 250UL
#define BLYNK_STOCK_RECOVERY_WATCHDOG_MS 2000UL

// v3.9.175: an authenticated login is not sufficient evidence of a working
// bidirectional session. Capture the library's inbound-activity value at the
// connection edge and require it to change afterwards. This observes protocol
// traffic inside BlynkProtocol rather than relying on the diagnostic socket
// peek, which can miss a header consumed in the same run() call.
#define BLYNK_LOGIN_ONLY_MAX_SESSION_MS \
    (BLYNK_HEARTBEAT * 1000UL + BLYNK_TIMEOUT_MS * 3UL + 10000UL)
#define BLYNK_LOGIN_ONLY_RECOVERY_THRESHOLD 3U
static uint32_t blynkSessionInboundBaseline = 0;
static uint32_t blynkSessionWiFiDisconnectSeq = 0;
static bool blynkSessionSawPostLoginInbound = false;
static uint8_t blynkConsecutiveLoginOnlyLosses = 0;
// Set while a confirmed sequence-hole replacement socket is being verified.
// Application publishers remain silent until the protocol-only session proves
// bidirectional for the full verification interval.
static bool blynkSequenceHoleRecoveryTrial = false;
static unsigned long blynkSequenceHoleRecoveryStartedMillis = 0;
static uint8_t blynkSequenceHoleSocketTrialCount = 0;
static uint32_t blynkSequenceHoleServerCooldownCount = 0;
// Two independently verified sessions becoming one-way in a short window
// rules out one stale TCP sequence space. This is the v3.9.198 field pattern.
#define BLYNK_SEQUENCE_HOLE_RECURRENCE_WINDOW_MS 300000UL
static uint8_t blynkSequenceHoleRecentGenerations = 0;
static unsigned long blynkSequenceHoleLastGenerationMillis = 0;

static bool blynkSequenceHoleApplicationCanary = false;
static uint8_t blynkApplicationCanaryPassed = 0;
static uint32_t blynkApplicationCanarySentBaseline = 0;
static uint32_t blynkApplicationCanaryVerifiedBaseline = 0;

static const char* blynkStockSessionStateText(uint8_t state)
{
  switch (state) {
    case BLYNK_STOCK_SESSION_CONNECTING: return "CONNECTING";
    case BLYNK_STOCK_SESSION_ONLINE: return "ONLINE";
    case BLYNK_STOCK_SESSION_RECOVERY: return "RECOVERY";
    default: return "IDLE";
  }
}

static void blynkTxDropNonPriority();

static unsigned long blynkStockObservedResponseAgeMs(unsigned long now)
{
#if CONNECTION_MODE == 1
  // The forensic peek only sees RESPONSE headers that are pending at the
  // socket boundary; it is therefore an OBSERVED-response freshness metric,
  // not a direct Blynk PONG timestamp. A session-start value is seeded below
  // from the handshake response so normal startup does not look stale.
  return forensicRspLastNewMillis ? (unsigned long)(now - forensicRspLastNewMillis) : 0xFFFFFFFFUL;
#else
  (void)now;
  return 0UL;
#endif
}

static bool blynkStockApplicationTxAllowed(unsigned long now)
{
#if CONNECTION_MODE == 1
  // v3.9.186: an authenticated login is not enough to admit telemetry. During
  // a sequence-hole replacement trial, Blynk.run() alone owns the socket until
  // multiple protocol heartbeats have demonstrated a bidirectional stream.
  if (blynkSequenceHoleRecoveryTrial) return false;
  // v3.9.187: each application frame carries an atomically appended PING.
  // Admit no following virtualWrite until its exact RESPONSE proves ordered,
  // end-to-end delivery of the preceding frame.
  if (Blynk.applicationAckPending()) return false;
  // v3.9.192: during recovery, admit exactly one application frame, then wait
  // for the library's partial-batch PING RESPONSE before admitting the next.
  // The connected-session service advances these baselines after each proof.
  if (blynkSequenceHoleApplicationCanary &&
      Blynk.applicationAckSentCount() > blynkApplicationCanarySentBaseline) {
    return false;
  }
  // v3.9.147: the live stock session is the only transport authority. The
  // former sidecar probe tested a different TCP connection, so it could pass
  // while this session was stale or fail while this session was healthy.
  // Keep only the brief post-PING quiet window.
  if (now < blynkStockTxPingQuietUntil) return false;
  // v3.9.162: do not add dashboard/sensor frames while a prior Blynk frame is
  // waiting for lwIP to regain send-buffer space. Blynk.run() and protocol
  // PING/RESPONSE traffic remain active, and V61/V62 stay in the application
  // coalescing queue with their immutable common timestamp.
  if (_blynkWifiClient.pendingBytes() != 0 ||
      _blynkWifiClient.awaitingPeerConfirmation()) return false;
  // v3.9.184: reserve a quiet interval before the protocol-owned 10-second
  // PING. Application frames previously ran until only 74ms before the first
  // missed PING. lastInboundActivity advances on its RESPONSE, so allowing TX
  // only during the first 6s of each healthy interval creates a real ~4s quiet
  // window without adding another heartbeat owner.
  const uint32_t protocolInboundAge =
      (uint32_t)(now - Blynk.lastInboundActivity());
  if (Blynk.connected() &&
      protocolInboundAge >= BLYNK_STOCK_PRE_PING_QUIET_RX_MS) {
    if (protocolInboundAge >= BLYNK_STOCK_APPLICATION_TX_PAUSE_RX_MS &&
        !blynkStockRxTrafficBrakeActive) {
      blynkStockRxTrafficBrakeActive = true;
      Serial.printf("[Blynk TX BRAKE] inbound stale %lums; pausing application writes while TCP/PING recovery continues\n",
                    (unsigned long)protocolInboundAge);
    }
    return false;
  }
  if (blynkStockRxTrafficBrakeActive) {
    blynkStockRxTrafficBrakeActive = false;
    Serial.printf("[Blynk TX BRAKE] fresh inbound response; application writes resumed after %lums\n",
                  (unsigned long)protocolInboundAge);
  }
  return Blynk.connected() && blynkStockSessionState == BLYNK_STOCK_SESSION_ONLINE;
#else
  (void)now;
  return true;
#endif
}

static void blynkTxDropCoherentPair()
{
  for (uint8_t i = 0; i < BLYNK_TX_QUEUE_SIZE; ++i) {
    if (blynkTxQueue[i].pending &&
        (blynkTxQueue[i].vpin == V61 || blynkTxQueue[i].vpin == V62)) {
      blynkTxQueue[i].pending = false;
    }
  }
  blynkCoherentPairActive = false;
  blynkCoherentPairSecondDue = false;
  blynkCoherentPairDeviceMs = 0;
  blynkCoherentPairGpsTs = 0;
  blynkCoherentPairSequence = 0;
}

static void blynkStockEnterRecovery(uint32_t reason)
{
#if CONNECTION_MODE == 1
  // v3.9.101: retained only as a compatibility helper for historical code.
  // It deliberately does NOT call Blynk.disconnect(), Blynk.config(), or stop
  // the stock client.  The stock Blynk protocol owns connection recovery.
  const unsigned long now = millis();
  blynkStockLastRecoveryReason = reason;
  ++blynkStockRecoveryCount;
  blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
  blynkStockStateSinceMillis = now;
  blynkStockNextRunMillis = now + BLYNK_STOCK_RETRY_INTERVAL_MS;
  blynkOfflineSince = now;
  blynkTxDropCoherentPair();
  DBG_PRINTF("[Blynk SIMPLE] passive recovery reason=%lu — stock Blynk owns reconnect\n",
             (unsigned long)reason);
#endif
}


static uint16_t blynkTxPendingCount()
{
  uint16_t count = 0;
  for (uint8_t i = 0; i < BLYNK_TX_QUEUE_SIZE; ++i) {
    if (blynkTxQueue[i].pending) ++count;
  }
  return count;
}

static bool blynkTxCongested(uint16_t limit = BLYNK_TX_SOFT_LIMIT)
{
  return blynkTxPendingCount() >= limit;
}

static bool blynkTxIsPriorityVPin(uint8_t vpin)
{
  return vpin == V4 || vpin == V61 || vpin == V62;
}

// v3.9.92: when an established session has already suffered a long socket
// write, do not carry a stale backlog of dashboard/diagnostic values into the
// next connection. Preserve only the newest authoritative V4/V61/V62 records.
static void blynkTxDropNonPriority()
{
  for (uint8_t i = 0; i < BLYNK_TX_QUEUE_SIZE; ++i) {
    if (blynkTxQueue[i].pending && !blynkTxIsPriorityVPin(blynkTxQueue[i].vpin)) {
      blynkTxQueue[i].pending = false;
      ++blynkTxCongestionSkips;
    }
  }
}

// v3.9.175: this is intentionally a notification-only compatibility helper.
// The old implementation walked tcp_active_pcbs/tcp_tw_pcbs and called
// tcp_abort() directly. That is invalid for a PCB owned by lwIP's BSD
// socket/netconn layer: NetworkClient still owns the descriptor and must be the
// component that releases it. Every caller invokes Blynk.disconnect()/stop()
// first or is reached after BlynkProtocol has done so; no raw PCB action is
// permitted here.
static void blynkPurgeClosedSessionPcbs(const char *source)
{
  Serial.printf("[Blynk SOCKET CLEANUP] source=%s normal owner close only; raw PCB abort disabled\n",
                source ? source : "UNKNOWN");
}

// Count only the repeatable field failure: login succeeded, no subsequent
// inbound protocol activity was observed, the session reached its bounded
// heartbeat deadline, and WiFi did
// not report a disconnect. This avoids treating server maintenance, a real WiFi
// loss, or a long otherwise-working session as the lower-stack poisoned state.
static void blynkRecordSessionLossAndMaybeRestart(const char *source)
{
  const uint32_t now = millis();
  const uint32_t sessionAge = now - blynkLastConnectedMillis;
  const bool wifiUnchanged =
      WiFi.status() == WL_CONNECTED &&
      wifiDisconnectEventSequence == blynkSessionWiFiDisconnectSeq;
  const bool loginOnlyFailure =
      blynkLastConnectedMillis != 0 &&
      !blynkSessionSawPostLoginInbound &&
      sessionAge <= BLYNK_LOGIN_ONLY_MAX_SESSION_MS &&
      wifiUnchanged;

  if (!loginOnlyFailure) {
    // The fail-safe requires consecutive instances of the same proven pattern.
    // A real WiFi event, a long session, or observed post-login traffic breaks
    // that sequence.
    blynkConsecutiveLoginOnlyLosses = 0;
    return;
  }

  if (blynkConsecutiveLoginOnlyLosses < 255U) {
    ++blynkConsecutiveLoginOnlyLosses;
  }
  Serial.printf("[Blynk LOGIN-ONLY STALL] source=%s streak=%u/%u sessionAge=%lums inboundBaseline=%lu inboundNow=%lu WiFi=%d wifiDiscSeq=%lu\n",
                source ? source : "UNKNOWN",
                (unsigned)blynkConsecutiveLoginOnlyLosses,
                (unsigned)BLYNK_LOGIN_ONLY_RECOVERY_THRESHOLD,
                (unsigned long)sessionAge,
                (unsigned long)blynkSessionInboundBaseline,
                (unsigned long)Blynk.lastInboundActivity(),
                (int)WiFi.status(),
                (unsigned long)wifiDisconnectEventSequence);

  if (blynkConsecutiveLoginOnlyLosses >= BLYNK_LOGIN_ONLY_RECOVERY_THRESHOLD) {
    // A login-only failure proves only that the peer stopped completing the
    // protocol. Rebooting the whole sensor cannot repair a remote server and
    // discards otherwise healthy acquisition state. Let the bounded stock
    // retry/clean-refresh state machine continue instead.
    Serial.println("[Blynk LOGIN-ONLY STALL] threshold reached; preserving MCU/WiFi and continuing protocol-only recovery");
    blynkConsecutiveLoginOnlyLosses = 0;
  }
}

// v3.9.119: Blynk v0.6.1 protocol-state reset helper.
//
// Important: Blynk.disconnect() puts BlynkProtocol into DISCONNECTED state,
// and Blynk.config() only changes the endpoint/configuration; it does NOT put
// the protocol back into CONNECTING. In v0.6.1, Blynk.connect(0) is the
// non-blocking way to restore CONNECTING without entering the blocking
// connect/run loop. Normal Blynk.run() remains the sole connection owner.
static void blynkStockProtocolReset()
{
#if CONNECTION_MODE == 1
  Blynk.disconnect();
  blynkPurgeClosedSessionPcbs("protocol-reset");
  delay(5);
  Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
  blynkConfigured = true;
  // Blynk v0.6.1: timeout=0 performs conn.disconnect()+state=CONNECTING
  // but skips the potentially blocking while(run()) loop.
  (void)Blynk.connect(0);
#endif
}

static void blynkStockEnterTxQuarantine(uint8_t reason, unsigned long stallMs)
{
#if CONNECTION_MODE == 1
  const unsigned long now = millis();
  // A returned write >=750ms is outside the intended Core-1 network budget,
  // but elapsed time alone does not prove that the socket failed. In field
  // logs virtualWrite() returned with Blynk.connected()==true and this helper
  // itself caused the immediately following disconnect. Hold only application
  // traffic and keep servicing the existing protocol session with Blynk.run().
  blynkStockTxRecoveryPending = false;
  blynkStockTxQuarantineUntil = now + BLYNK_STOCK_TX_QUARANTINE_MS;
  blynkStockStableUntilMillis = blynkStockTxQuarantineUntil;
  blynkTxDropNonPriority();
  blynkTxDropCoherentPair();
  blynkStockLastTxProbeOkMillis = 0;
  blynkStockTxProbeInFlight = false;
  blynkStockTxProbeFailures = 0;
  DBG_PRINTF("[Blynk TX] BACKOFF reason=%u returned write=%lums — application TX paused; live Blynk session kept\n",
             (unsigned)reason, stallMs);
  blynkTxGraceUntil = now + BLYNK_STOCK_TX_QUARANTINE_MS;
#endif
}

static void blynkTxService()
{
#if CONNECTION_MODE == 1
  if (blynkStockSessionState != BLYNK_STOCK_SESSION_ONLINE) return;
  if (millis() < blynkStockTxQuarantineUntil) return;
#else
  if (!Blynk.connected()) return;
#endif
  if (millis() < blynkTxGraceUntil) return;

  const unsigned long now = millis();
  if (!blynkStockApplicationTxAllowed(now)) return;
  // v3.9.193: the field failure begins while RSSI is falling through -69 dBm.
  // Reduce RF/TCP queue pressure before the association itself is lost. Keep
  // hysteresis so normal RSSI jitter cannot repeatedly enter/leave this mode.
  const int32_t linkRssi = WiFi.RSSI();
  if (!blynkWeakLinkMode && linkRssi <= BLYNK_WEAK_LINK_ENTER_RSSI_DBM) {
    blynkWeakLinkMode = true;
    if (blynkWeakLinkEntries < 0xFFFFFFFFUL) ++blynkWeakLinkEntries;
    Serial.printf("[Blynk LINK SHAPING] ENTER RSSI=%lddBm; retaining V4/V61/V62, application pace=%lums entries=%lu\n",
                  (long)linkRssi,
                  (unsigned long)BLYNK_WEAK_LINK_TX_INTERVAL_MS,
                  (unsigned long)blynkWeakLinkEntries);
  } else if (blynkWeakLinkMode && linkRssi >= BLYNK_WEAK_LINK_EXIT_RSSI_DBM) {
    blynkWeakLinkMode = false;
    Serial.printf("[Blynk LINK SHAPING] EXIT RSSI=%lddBm; normal application schedule restored\n",
                  (long)linkRssi);
  }
  if (blynkWeakLinkMode) blynkTxDropNonPriority();
  // V62 is the second half of an already-started database record. It remains
  // the next application write, but still observes the established transport
  // transport pacing. The immutable timestamp, not arrival time, joins it.
  const unsigned long txInterval = blynkWeakLinkMode
      ? BLYNK_WEAK_LINK_TX_INTERVAL_MS : BLYNK_TX_INTERVAL_MS;
  if (now - blynkTxLastSend < txInterval) return;

  if (blynkTxPendingCount() >= BLYNK_TX_HARD_LIMIT) {
    blynkTxDropNonPriority();
  }

  // v3.9.91: reserve the next transmit opportunities for the authoritative
  // 1-second records. V4 is the dashboard heartbeat/value; V61+V62 are the
  // two halves of one coherent database/GIS snapshot. Ordinary dashboard and
  // diagnostic traffic never gets ahead of these three records.
  const uint8_t priorityPins[] = { V4, V61, V62 };
  const uint8_t priorityStart = blynkCoherentPairSecondDue ? 2U : 0U;
  for (uint8_t p = priorityStart; p < sizeof(priorityPins); ++p) {
    const uint8_t priorityVPin = priorityPins[p];
    for (uint8_t n = 0; n < BLYNK_TX_QUEUE_SIZE; ++n) {
      uint8_t i = (uint8_t)((blynkTxNext + n) % BLYNK_TX_QUEUE_SIZE);
      if (!blynkTxQueue[i].pending || blynkTxQueue[i].vpin != priorityVPin) continue;

      blynkTxQueue[i].pending = false;
      blynkTxNext = (uint8_t)((i + 1) % BLYNK_TX_QUEUE_SIZE);
      blynkTxLastSend = now;

      const uint8_t txVPin = blynkTxQueue[i].vpin;
      const uint16_t txLen = (uint16_t)strlen(blynkTxQueue[i].value);
      const uint16_t pendingBefore = blynkTxPendingCount();
      const unsigned long txStart = millis();
      blynkTxCallCount++;
      blynkLastTxAttemptMillis = millis();
      blynkTxLastVPin = txVPin;
      blynkTxLastValueLen = txLen;
      core1StageBegin(CORE1_STAGE_BLYNK_TX_CALL);
      DBG_PRINTF("[Blynk TX] START PRIORITY V%d len=%u pending=%u slot=%u\n",
                 txVPin, (unsigned)txLen, (unsigned)pendingBefore, (unsigned)i);
      if (txVPin == V61) {
        const uint32_t txPairSequence = blynkCoherentPairSequence;
        v61TestLastStartMillis = txStart;
        Serial.printf("[V61 TX] START pair=%lu key=%ld/%lu len=%u pending=%u at=%lums\n",
                      (unsigned long)txPairSequence,
                      blynkCoherentPairGpsTs, blynkCoherentPairDeviceMs,
                      (unsigned)txLen, (unsigned)pendingBefore, txStart);
      }
      if (blynkSequenceHoleApplicationCanary) {
        Blynk.forceNextApplicationAck();
      }
      Blynk.virtualWrite(txVPin, blynkTxQueue[i].value);
      const unsigned long txElapsed = millis() - txStart;
      if (txVPin == V61) {
        const unsigned long endNow = millis();
        const unsigned long gap = (v61TestLastEndMillis == 0) ? 0 : (endNow - v61TestLastEndMillis);
        v61TestLastEndMillis = endNow;
        const bool stalled = (txElapsed >= BLYNK_STOCK_TX_STALL_MS);
        const bool notable = (txElapsed >= 500UL);
        Serial.printf("[V61 TX] END pair=%lu key=%ld/%lu elapsed=%lums gap=%lums connected=%d%s\n",
                      (unsigned long)blynkCoherentPairSequence,
                      blynkCoherentPairGpsTs, blynkCoherentPairDeviceMs,
                      txElapsed, gap,
                      Blynk.connected() ? 1 : 0, notable ? (stalled ? " *** TX_STALL ***" : " *** SLOW ***") : "");
        if (txElapsed < BLYNK_STOCK_TX_STALL_MS) {
          blynkCoherentPairSecondDue = true;
        }
      }
      if (txElapsed < BLYNK_STOCK_TX_STALL_MS) blynkLastTxSuccessMillis = millis();
      blynkTxLastCallMs = txElapsed;
      if (txElapsed > blynkTxMaxCallMs) blynkTxMaxCallMs = txElapsed;
      DBG_PRINTF("[Blynk TX] END PRIORITY V%d duration=%lums pendingAfter=%u max=%lums\n",
                 txVPin, txElapsed, (unsigned)blynkTxPendingCount(),
                 (unsigned long)blynkTxMaxCallMs);
      if (txVPin == V62 && txElapsed < BLYNK_STOCK_TX_STALL_MS) {
        Serial.printf("[V61/V62 PAIR] COMPLETE pair=%lu key=%ld/%lu V62elapsed=%lums\n",
                      (unsigned long)blynkCoherentPairSequence,
                      blynkCoherentPairGpsTs, blynkCoherentPairDeviceMs,
                      txElapsed);
        blynkCoherentPairActive = false;
        blynkCoherentPairSecondDue = false;
        blynkCoherentPairDeviceMs = 0;
        blynkCoherentPairGpsTs = 0;
        blynkCoherentPairSequence = 0;
      }
      if (txElapsed >= BLYNK_STOCK_TX_STALL_MS) {
        // The stock write has already returned, so the only safe action here is to
        // latch recovery and let the outer supervisor tear down the session and schedule a fresh
        // stock Blynk.run() handshake.  This is deliberately Blynk-only: WiFi
        // remains associated unless the WiFi state machine independently says
        // that WiFi itself is unhealthy.
        blynkStockEnterTxQuarantine(1, txElapsed);
      }
      core1StageEnd();
      return;
    }
  }

  for (uint8_t n = 0; n < BLYNK_TX_QUEUE_SIZE; ++n) {
    uint8_t i = (uint8_t)((blynkTxNext + n) % BLYNK_TX_QUEUE_SIZE);
    if (!blynkTxQueue[i].pending) continue;

    // Mark the slot consumed before sending.  If the connection drops while
    // sending, the next timer callback will queue a fresh/current value.
    blynkTxQueue[i].pending = false;
    blynkTxNext = (uint8_t)((i + 1) % BLYNK_TX_QUEUE_SIZE);
    blynkTxLastSend = now;

    const uint8_t txVPin = blynkTxQueue[i].vpin;
    const uint16_t txLen = (uint16_t)strlen(blynkTxQueue[i].value);
    const uint16_t pendingBefore = blynkTxPendingCount();
    const unsigned long txStart = millis();

    blynkTxCallCount++;
    blynkLastTxAttemptMillis = millis();
    blynkTxLastVPin = txVPin;
    blynkTxLastValueLen = txLen;

    core1StageBegin(CORE1_STAGE_BLYNK_TX_CALL);
    DBG_PRINTF("[Blynk TX] START V%d len=%u pending=%u slot=%u\n",
                  txVPin, (unsigned)txLen, (unsigned)pendingBefore, (unsigned)i);

    if (blynkSequenceHoleApplicationCanary) {
      Blynk.forceNextApplicationAck();
    }
    Blynk.virtualWrite(txVPin, blynkTxQueue[i].value);
    const unsigned long txElapsed = millis() - txStart;
    if (txElapsed < BLYNK_STOCK_TX_STALL_MS) blynkLastTxSuccessMillis = millis();
    blynkTxLastCallMs = txElapsed;
    if (txElapsed > blynkTxMaxCallMs) blynkTxMaxCallMs = txElapsed;
    DBG_PRINTF("[Blynk TX] END   V%d duration=%lums pendingAfter=%u max=%lums\n",
                  txVPin, txElapsed, (unsigned)blynkTxPendingCount(),
                  (unsigned long)blynkTxMaxCallMs);
    if (txElapsed >= BLYNK_STOCK_TX_STALL_MS) {
      blynkStockEnterTxQuarantine(2, txElapsed);
    }
    core1StageEnd();
    return;  // exactly one packet per service pass
  }

  if (blynkMapPending) {
    blynkMapPending = false;
    const unsigned long txStart = millis();
    blynkTxCallCount++;
    blynkLastTxAttemptMillis = millis();
    blynkTxLastVPin = V33;
    blynkTxLastValueLen = 0;

    core1StageBegin(CORE1_STAGE_BLYNK_MAP_TX_CALL);
    DBG_PRINTF("[Blynk TX MAP] START V33 pending=%u index=%ld\n",
                  (unsigned)blynkTxPendingCount(), blynkMapIndex);

    Blynk.virtualWrite(V33, blynkMapIndex, blynkMapLat, blynkMapLng, "AOQ-ESP32");
    const unsigned long txElapsed = millis() - txStart;
    if (txElapsed < BLYNK_STOCK_TX_STALL_MS) {
      blynkLastTxSuccessMillis = millis();
    }
    if (txElapsed >= BLYNK_STOCK_TX_STALL_MS) {
      blynkStockEnterTxQuarantine(3, txElapsed);
    }


    blynkTxLastCallMs = txElapsed;
    if (txElapsed > blynkTxMaxCallMs) blynkTxMaxCallMs = txElapsed;
    DBG_PRINTF("[Blynk TX MAP] END   V33 duration=%lums max=%lums\n",
                  txElapsed, (unsigned long)blynkTxMaxCallMs);
    core1StageEnd();
    return;
  }
}

// ============================================================================
// GPS — ATGM336H / AT6558 (NMEA + PCAS configuration)
// ============================================================================

void initGPS()
{
  // ATGM336H can output NMEA at various baud rates. Auto-detect to handle
  // modules configured for different rates. Common rates: 9600, 38400, 57600, 115200.
  //
  // v3.9.32: Wait for PPS signal (V26=1) before starting baud detection to ensure
  // GPS has initialized and is transmitting data. 30-second timeout with fallback.
  // After baud detection, configure GPS to 115200 baud and 10Hz update rate via PCAS commands.
  //
  // IMPORTANT for Arduino-ESP32 2.0.17: enlarge the UART1 RX ring BEFORE
  // begin().  The GPS can emit several NMEA sentences back-to-back while the
  // sensor task is busy with I2C/ADC work.  The old default buffer is small
  // enough that complete sentences can be dropped.  This is especially
  // important here because PPS is independent of the NMEA UART stream.
  // Explicitly establish the UART1 pin routing and standard UART polarity.
  // Arduino-ESP32 2.0.17 supports setPins() before/after begin(); begin()
  // below still receives the pins explicitly as the primary configuration.
  // Do NOT call pinMode() on these pins afterwards: that would be a GPIO
  // configuration layered on top of the UART pin-matrix routing.
  
  gps_serial.setRxBufferSize(4096);
  
  // Wait for PPS signal before starting baud detection (GPS needs time to initialize)
  sensorLogPrintf("[INIT] GPS waiting for PPS signal (V26=1) before baud detection...\n");
  unsigned long ppsWaitStart = millis();
  const unsigned long PPS_WAIT_TIMEOUT_MS = 3000; // 30 second timeout for PPS
  bool ppsDetected = false;
  
  while (millis() - ppsWaitStart < PPS_WAIT_TIMEOUT_MS) {
    if (ppsIsLocked()) {
      ppsDetected = true;
      sensorLogPrintf("[INIT] PPS signal detected after %lums - starting baud detection\n", 
                   (unsigned long)(millis() - ppsWaitStart));
      break;
    }
    delay(100);
  }
  
  if (!ppsDetected) {
    sensorLogPrintf("[INIT] PPS timeout - proceeding with baud detection anyway\n");
  }
  
  // Try common GPS baud rates to find the correct one
  const unsigned long baudRates[] = {9600, 38400, 57600, 115200};
  const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);
  unsigned long detectedBaud = GPS_BAUD; // Default to configured rate
  
  sensorLogPrintf("[INIT] GPS auto-baud detection started...\n");
  
  for (int i = 0; i < numBaudRates; i++) {
    unsigned long baud = baudRates[i];
    sensorLogPrintf("[INIT] Trying GPS baud rate: %lu... ", baud);
    
    // End any existing connection
    gps_serial.end();
    delay(50);
    
    // Try this baud rate
    gps_serial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gps_serial.setPins(GPS_RX_PIN, GPS_TX_PIN);
    gps_serial.setRxInvert(false);
    gps_serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
    gps_serial.setTimeout(20);
    delay(100);
    
    // Read some data and check if it looks like valid NMEA
    bool validNMEA = false;
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) { // 1 second to collect data (PPS already detected)
      while (gps_serial.available() > 0) {
        int c = gps_serial.read();
        if (c == '$') {
          // Found start of NMEA sentence, check next chars for ASCII
          int validChars = 0;
          for (int j = 0; j < 10 && gps_serial.available() > 0; j++) {
            int next = gps_serial.read();
            if ((next >= 32 && next <= 126) || next == '\r' || next == '\n') {
              validChars++;
            }
          }
          if (validChars >= 5) {
            validNMEA = true;
            break;
          }
        }
      }
      if (validNMEA) break;
      delay(10);
    }
    
    if (validNMEA) {
      detectedBaud = baud;
      sensorLogPrintf("SUCCESS - valid NMEA detected\n");
      break;
    } else {
      sensorLogPrintf("no valid NMEA\n");
    }
  }
  
  // Reinitialize with detected baud rate
  gps_serial.end();
  delay(50);
  gps_serial.begin(detectedBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gps_serial.setPins(GPS_RX_PIN, GPS_TX_PIN);
  gps_serial.setRxInvert(false);
  gps_serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
  gps_serial.setTimeout(20);
  delay(100);
  
  sensorLogPrintf("[INIT] GPS UART1 initially at: RX=GPIO%d TX=GPIO%d baud=%lu (auto-detected)\n",
                GPS_RX_PIN, GPS_TX_PIN, (unsigned long)gps_serial.baudRate());

  // v3.9.50 CORRECTION: the v3.9.48/v3.9.49 PMTK-based approach was not
  // the correct command family for this ATGM336H. The historical notes below
  // are retained, but the actual command sent here is the native PCAS03 command.
  // PCAS03 is the ATGM336H/AT6558 NMEA sentence-selection command.

  // v3.9.48 FIX: PMTK314 (limit NMEA output to RMC+GGA only) moved here —
  // sent immediately after auto-detection, at detectedBaud, BEFORE the baud-
  // switch ladder — instead of its previous position after the ladder
  // completed. This reverses the sequence used in v3.9.44-47, which followed
  // generic "change baud before rate/filter" guidance from other MTK-family
  // module reports. That guidance assumes a module that reliably receives
  // commands in the first place; it does not address a module whose DEFAULT
  // output (every sentence type, not just RMC+GGA) is heavy enough at 9600
  // baud to interfere with receiving new commands at all.
  // Field evidence for this: with the previous ordering, our attempt to read
  // the module's PMTK251 ACK response captured plain GGA/GSV fragments (e.g.
  // "$GNGGA,185249.000,,,,,0,...", "8,,25,63,270,,28,20,304,,0*67") instead
  // of an actual $PMTK001,251,x ACK sentence — meaning the module's transmit
  // buffer was so busy with its full default sentence set that our listen
  // window only ever caught regular traffic, on every attempt, at every
  // candidate baud rate, which explains why 115200 and 57600 failed
  // identically (a real baud-specific limit would not fail the same way at
  // two different rates). Sending PMTK314 first — while still at the
  // confirmed-working detectedBaud — cuts the module's own output load
  // before we ask it to also process a baud change, giving it much more
  // room to actually receive and act on PMTK251 afterward.
  // v3.9.51 OVERRIDE: configuration/persistence is now deliberately one-time.
  // Only an EXACT 9600-baud detection triggers GPS configuration. If the GPS is
  // already operating above 9600, we assume its stored configuration is already
  // in the desired higher-speed state and leave it completely untouched.
  // Therefore NO PCAS03, PCAS01, PCAS02, or PCAS00 command is sent on normal
  // boots where the receiver is already above 9600 baud.
  //
  // This prevents unnecessary flash writes and prevents the firmware from
  // needlessly changing an already-configured receiver on every ESP32 boot.
  unsigned long settledBaud = detectedBaud;
  bool verifiedAtTarget = false;

  if (detectedBaud > 9600UL) {
    sensorLogPrintf("[INIT] GPS already above 9600 baud (%lu) — assuming configuration is already saved; skipping PCAS configuration and PCAS00 save\n",
                  detectedBaud);
    settledBaud = detectedBaud;
    verifiedAtTarget = true;
  } else if (detectedBaud == 9600UL) {
    // ONLY this path changes the GPS configuration and writes it to flash.
    // Migrate the receiver from the known 9600-baud state to 115200 + 10Hz.

    sensorLogPrintf("[INIT] GPS detected at 9600 baud — migrating to 115200 + 10Hz and saving configuration...\n");

    // Reduce NMEA output before changing baud. PCAS03 is the native
    // ATGM336H/AT6558 sentence-selection command.
    sensorLogPrintf("[INIT] Configuring ATGM336H NMEA output via PCAS03 (GGA+RMC only) BEFORE baud switch...\n");
    const char* nmeaCmd = "$PCAS03,1,0,0,0,1,0,0,0*02\r\n";
    gps_serial.print(nmeaCmd);
    gps_serial.flush();
    delay(200);

    const unsigned long targetBaud = 115200UL;
    const char* baudCmd = "$PCAS01,5*19\r\n";
    const int maxAttempts = 2;

    sensorLogPrintf("[INIT] Configuring GPS baud rate to %lu via PCAS01...\n", targetBaud);

    for (int attempt = 1; attempt <= maxAttempts && !verifiedAtTarget; attempt++) {
      sensorLogPrintf("[INIT] GPS baud %lu switch attempt %d/%d...\n",
                    targetBaud, attempt, maxAttempts);

      // Always send PCAS01 at the confirmed-working 9600 baud.
      if (gps_serial.baudRate() != detectedBaud) {
        gps_serial.end();
        delay(50);
        gps_serial.begin(detectedBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        gps_serial.setPins(GPS_RX_PIN, GPS_TX_PIN);
        gps_serial.setRxInvert(false);
        gps_serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
        gps_serial.setTimeout(20);
        delay(50);
      }

      gps_serial.print(baudCmd);
      gps_serial.flush();
      delay(50);
      delay(250UL * attempt);

      // Switch the ESP32 UART to the requested GPS baud and verify real NMEA.
      gps_serial.end();
      delay(50);
      gps_serial.begin(targetBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
      gps_serial.setPins(GPS_RX_PIN, GPS_TX_PIN);
      gps_serial.setRxInvert(false);
      gps_serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
      gps_serial.setTimeout(20);
      delay(100);

      sensorLogPrintf("[INIT] GPS UART1 reconfigured to baud=%lu (attempt %d)\n",
                    (unsigned long)gps_serial.baudRate(), attempt);

      unsigned long verifyStart = millis();
      while (millis() - verifyStart < 2500UL) {
        while (gps_serial.available() > 0) {
          int c = gps_serial.read();
          if (c == '$') {
            int validChars = 0;
            for (int j = 0; j < 10 && gps_serial.available() > 0; j++) {
              int next = gps_serial.read();
              if ((next >= 32 && next <= 126) || next == '\r' || next == '\n') validChars++;
            }
            if (validChars >= 5) {
              verifiedAtTarget = true;
              break;
            }
          }
        }
        if (verifiedAtTarget) break;
        delay(10);
      }

      if (verifiedAtTarget) {
        settledBaud = targetBaud;
        sensorLogPrintf("[INIT] GPS baud switch VERIFIED at %lu\n", targetBaud);
      } else {
        sensorLogPrintf("[INIT] GPS baud %lu switch attempt %d/%d NOT verified\n",
                      targetBaud, attempt, maxAttempts);

        // Return to the confirmed-working 9600 before retrying.
        gps_serial.end();
        delay(50);
        gps_serial.begin(detectedBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        gps_serial.setPins(GPS_RX_PIN, GPS_TX_PIN);
        gps_serial.setRxInvert(false);
        gps_serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
        gps_serial.setTimeout(20);
        delay(100);
      }
    }

    if (!verifiedAtTarget) {
      // Do NOT save a partially-applied configuration. Keep the known-working
      // 9600 baud and leave flash unchanged.
      settledBaud = detectedBaud;
      sensorLogPrintf("[INIT] GPS 115200 migration failed — keeping 9600; NO PCAS02 and NO PCAS00 save\n");
    } else {
      // Only after 115200 is positively verified do we request 10Hz.
      sensorLogPrintf("[INIT] Configuring GPS update rate to 10Hz via PCAS02...\n");
      const char* rateCmd = "$PCAS02,100*1E\r\n";
      gps_serial.print(rateCmd);
      gps_serial.flush();
      delay(150);

      // PCAS00 is ONLY sent after the successful 9600 -> 115200 migration.
      sensorLogPrintf("[INIT] Saving ATGM336H configuration to flash via PCAS00 (one-time migration)...\n");
      const char* saveCmd = "$PCAS00*01\r\n";
      gps_serial.print(saveCmd);
      gps_serial.flush();
      delay(200);
    }
  } else {
    // Defensive fallback for an unexpected baud below 9600. Leave the receiver
    // untouched rather than issuing an unsolicited flash write.
    sensorLogPrintf("[INIT] GPS detected at unexpected baud %lu — leaving configuration unchanged; NO PCAS00 save\n",
                  detectedBaud);
    settledBaud = detectedBaud;
    verifiedAtTarget = true;
  }

  const bool gpsUse10Hz = (settledBaud >= 19200UL);

  gpsUartBytesReceived = 0;
  gpsUartBytesThisBoot = 0;
  gpsLastByteMillis = millis();

  sensorLogPrintf("[INIT] GPS UART1 final config: RX=GPIO%d TX=GPIO%d baud=%lu 8N1 invert=OFF flow=OFF\n",
                GPS_RX_PIN, GPS_TX_PIN, (unsigned long)gps_serial.baudRate());

  // v3.9.51: reports the actual live baud AND the expected configured update
  // rate. A receiver already above 9600 is not modified or flash-saved.
  engMsgf("GPS: ATGM336H UART1 RX13/TX23 configured to %lu baud + %s via PCAS — RX buffer 4096 — NMEA parser diagnostics enabled",
          (unsigned long)gps_serial.baudRate(), gpsUse10Hz ? "10Hz" : "5Hz");
}

void feedGPS()
{
  uint32_t perfGpsStartUs = micros();

  unsigned long t = millis();
  while (millis() - t < GPS_FEED_MS) {
    int availableNow = gps_serial.available();
    while (availableNow-- > 0) {
      int c = gps_serial.read();
      if (c < 0) break;
      gpsUartBytesReceived++;
      gpsUartBytesThisBoot++;
      gpsLastByteMillis = millis();
      if (c == '$') gpsDollarCount++;
      if (c == '\n') gpsCrlfCount++;
      gpsRawCaptureByteSafe((uint8_t)c);
      gps.encode((char)c);
    }
    taskYIELD();
  }

  // GPS DEBUG ONLY — no effect on GPS processing or normal operation.
  // v3.9.32: the concise UART status is printed even when DEBUGON is false,
  // because V12/V13/V14/V17 being all zero/99.9 while V26=1 proves that PPS
  // is alive but does not prove that the NMEA UART is reaching GPIO13.
  // This lets us distinguish: no UART bytes, wrong baud/garbage, or valid NMEA
  // that TinyGPS++ has not yet converted into a fix. Baud rate is now auto-detected.
  {
    if (GPS_DEBUG_PRINTOUT == true) {
      static unsigned long lastGPSUartStatus = 0;
      if (millis() - lastGPSUartStatus >= 5000) {
        lastGPSUartStatus = millis();
        sensorLogPrintf("[GPS UART] baud=%lu RX13/TX23 bytes=%lu chars=%lu $=%lu LF=%lu pass=%lu fail=%lu fixSent=%lu sats=%lu hdop=%.2f loc=%s lastByteAge=%lums PPS=%d rawLines=%u\n",
                      (unsigned long)gps_serial.baudRate(),
                      (unsigned long)gpsUartBytesReceived,
                      gps.charsProcessed(),
                      (unsigned long)gpsDollarCount,
                      (unsigned long)gpsCrlfCount,
                      gps.passedChecksum(),
                      gps.failedChecksum(),
                      gps.sentencesWithFix(),
                      gps.satellites.value(),
                      gps.hdop.value()/100.0f,
                      gps.location.isValid() ? "VALID" : "INVALID",
                      gpsLastByteMillis ? (unsigned long)(millis() - gpsLastByteMillis) : 0UL,
                      ppsIsLocked() ? 1 : 0,
                      (unsigned)gpsRawLineCount);
        gpsPrintRawCapture();
      }
    }
  }

  if (DEBUGON) {
    static unsigned long lastGPSDebug = 0;
    if (millis() - lastGPSDebug >= 5000) {
      lastGPSDebug = millis();
      DBG_PRINTF("[GPS DEBUG] uartBytes=%lu chars=%lu sats=%lu hdop=%.2f location=%s age=%lu ms lastByte=%lums ago\n",
                    (unsigned long)gpsUartBytesReceived,
                    gps.charsProcessed(),
                    gps.satellites.value(),
                    gps.hdop.value()/100.0f,
                    gps.location.isValid() ? "VALID" : "INVALID",
                    gps.location.age(),
                    gpsLastByteMillis ? (unsigned long)(millis() - gpsLastByteMillis) : 0UL);
      DBG_PRINTF("[GPS DEBUG] lat=%.6f lng=%.6f fix=%s PPS=%d\n",
                    gps.location.isValid() ? gps.location.lat() : 0.0,
                    gps.location.isValid() ? gps.location.lng() : 0.0,
                    (gps.location.isValid() &&
                     gps.location.age() < GPS_TIMEOUT_MS &&
                     gps.satellites.value() >= 4) ? "VALID" : "INVALID",
                    ppsIsLocked() ? 1 : 0);
    }
  }

  perfRecordMax(perfMaxGpsFeedMs,
                (uint32_t)((micros() - perfGpsStartUs) / 1000UL));
}

// v2.5: Convert GPS UTC date+time (from NMEA RMC/GGA, already parsed by
// TinyGPS++) to a UNIX epoch timestamp — used as the point index for the
// Blynk Map widget on V33. Standard proleptic-Gregorian "days_from_civil"
// calendar math (Howard Hinnant's well-known constexpr algorithm); no RTC,
// no NTP, no external library needed — the GPS module is already an
// accurate UTC time source once it has a fix.
//
// IMPORTANT: this reads the global `gps` (TinyGPSPlus) object directly, so
// it must ONLY ever be called from sensorTask on Core 0 — the same core
// that owns and writes to `gps` via feedGPS(). Calling it from loop()/Core 1
// would be an unprotected cross-core access to a non-thread-safe object.
// The result is cached into sd.gps_unix_time (mutex-protected) so Core 1
// can read it safely without ever touching `gps` itself.
long gpsUnixTime()
{
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;
  int y  = gps.date.year();
  int m  = gps.date.month();
  int d  = gps.date.day();
  int hh = gps.time.hour();
  int mm = gps.time.minute();
  int ss = gps.time.second();
  if (y < 2020 || y > 2100) return 0;   // sanity guard against a bad/uninitialised NMEA fix

  y -= (m <= 2) ? 1 : 0;
  long era = (y >= 0 ? y : y - 399) / 400;
  long yoe = y - era * 400;
  long doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d - 1;
  long doe = yoe*365 + yoe/4 - yoe/100 + doy;
  long days = era*146097 + doe - 719468;
  return days*86400L + (long)hh*3600L + (long)mm*60L + (long)ss;
}

// ============================================================================
// ENS160 + AHT2x (I2C) — ScioSense_ENS160 library
// ============================================================================

bool initENS()
{
  // ScioSense API: ens160.begin() returns void; check ens160.available() after.
  ens160.begin();
  if (!ens160.available()) {
    engMsg("ENS160: not available (addr 0x53) — check wiring");
    return false;
  }
  // Set standard operating mode (continuous measurement)
  if (!ens160.setMode(ENS160_OPMODE_STD)) {
    engMsg("ENS160: setMode failed");
    return false;
  }
  // AHT2x init
  if (!aht.begin()) {
    engMsg("AHT2x: init failed — check wiring");
    return false;
  }
  engMsg("ENS160+AHT2x: OK");
  ens_i2c_fail_streak = 0;
  return true;
}

// v3.9.42 FIX: i2cLastSdaLevel/i2cLastSclLevel were previously declared much
// later in the file (after readENS(), which uses them in several DBG_PRINTF
// diagnostic lines). This compiled silently with DEBUGON=false because the
// no-op DBG_PRINTF macro discards its arguments entirely at the preprocessor
// stage, so the undeclared symbols were never actually reached by the
// compiler. Enabling DEBUGON=true for field diagnostics exposed the latent
// forward-reference bug. Moved the declarations here, before first use;
// the original declarations further down (with i2cLastWireError and the
// other I2C diagnostic counters, which are NOT used before their own
// declaration point) are left in place.
volatile uint8_t i2cLastSdaLevel = 1;
volatile uint8_t i2cLastSclLevel = 1;

// Read ENS160 + AHT2x. Returns true if valid data obtained.
// ENS160 needs temperature+humidity compensation for accuracy — we feed it
// from AHT2x which shares the same I2C bus on the same board.
// ScioSense API uses integer temp (°C) and humidity (%) for set_envdata().
bool readENS(float& tvoc, float& eco2, float& aqi_out, float& temp_out, float& hum_out)
{
  uint32_t perfEnsStartUs = micros();

  // Preflight both devices before entering their library transaction paths.
  // A mechanically intermittent connector should become a sensor fault, not
  // an extended I2C transaction that monopolises the sensor task.
  if (!i2cSensorPresent(0x53, "ENS160") || !i2cSensorPresent(0x38, "AHT2x")) {
    perfI2cError();
    perfRecordMax(perfMaxI2cPathMs,
                  (uint32_t)((micros() - perfEnsStartUs) / 1000UL));
    return false;
  }

  // Read AHT2x first — provides compensation values for ENS160
  sensors_event_t humEvent, tempEvent;
  uint32_t perfAht21StartUs = micros();
  perfI2cBegin(2, 1);
  bool perfAht21Ok = aht.getEvent(&humEvent, &tempEvent);
  uint32_t perfAht21ElapsedMs = (uint32_t)((micros() - perfAht21StartUs) / 1000UL);
  perfRecordMax(perfMaxAht21Ms, perfAht21ElapsedMs);
  if (perfAht21ElapsedMs >= I2C_DIAG_SLOW_MS) {
    DBG_PRINTF("[ENS160 DIAG] AHT2x getEvent time=%lums ok=%d SDA=%d SCL=%d\n",
               (unsigned long)perfAht21ElapsedMs, perfAht21Ok ? 1 : 0,
               i2cLastSdaLevel, i2cLastSclLevel);
  }
  if (!perfAht21Ok) {
    perfRecordMax(perfMaxI2cPathMs,
                  (uint32_t)((micros() - perfEnsStartUs) / 1000UL));
    return false;
  }
  temp_out = tempEvent.temperature;
  hum_out  = humEvent.relative_humidity;

  // Provide integer temp/humidity compensation to ENS160 for improved accuracy
  // ScioSense set_envdata() takes int (°C) and int (%) — cast from float
  uint32_t perfEns160OnlyStartUs = micros();
  perfI2cBegin(1, 3);
  uint32_t ensSetEnvStartUs = micros();
  ens160.set_envdata((int)temp_out, (int)hum_out);
  uint32_t ensSetEnvElapsedMs = (uint32_t)((micros() - ensSetEnvStartUs) / 1000UL);
  if (ensSetEnvElapsedMs >= I2C_DIAG_SLOW_MS) {
    DBG_PRINTF("[ENS160 DIAG] set_envdata time=%lums temp=%d hum=%d SDA=%d SCL=%d\n",
               (unsigned long)ensSetEnvElapsedMs, (int)temp_out, (int)hum_out,
               i2cLastSdaLevel, i2cLastSclLevel);
  }

  // Trigger measurement
  // v3.9.5 FIX: do NOT use measure(true)/measureRaw(true) here. The ScioSense
  // library implements those forms as an unbounded polling loop on DATA_STATUS.
  // If the ENS160 is electrically present but stops asserting NEWDAT/NEWGPR
  // (for example after a connector/wire disturbance), the sensorTask can stay
  // inside read8()/Wire indefinitely until the task-WDT fires. Instead, poll
  // with the library's non-waiting form and impose our own finite deadline.
  // Each individual Wire transaction remains bounded by Wire.setTimeOut(50),
  // while the outer deadline prevents the library's measurement wait from ever
  // becoming an unbounded sensor-task stall. Standard operating mode is
  // continuous, so a short wait here is enough; the next sensorTask cycle will
  // retry if fresh data has not appeared yet.
  perfI2cBegin(1, 2);
  bool ensMeasureReady = false;
  unsigned long ensMeasureWaitStart = millis();
  uint32_t ensMeasureCalls = 0;
  uint32_t ensMeasureSlowestMs = 0;
  while (millis() - ensMeasureWaitStart < ENS_MEASURE_TIMEOUT_MS) {
    uint32_t oneMeasureStartUs = micros();
    ensMeasureReady = ens160.measure(false);
    uint32_t oneMeasureElapsedMs = (uint32_t)((micros() - oneMeasureStartUs) / 1000UL);
    ++ensMeasureCalls;
    if (oneMeasureElapsedMs > ensMeasureSlowestMs) ensMeasureSlowestMs = oneMeasureElapsedMs;
    if (oneMeasureElapsedMs >= I2C_DIAG_SLOW_MS) {
      DBG_PRINTF("[ENS160 DIAG] measure(false) call=%lu time=%lums ready=%d SDA=%d SCL=%d\n",
                 (unsigned long)ensMeasureCalls, (unsigned long)oneMeasureElapsedMs,
                 ensMeasureReady ? 1 : 0, i2cLastSdaLevel, i2cLastSclLevel);
    }
    if (ensMeasureReady) break;
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  if (!ensMeasureReady) {
    DBG_PRINTF("[ENS160 DIAG] measure timeout calls=%lu slowest=%lums total=%lums SDA=%d SCL=%d\n",
               (unsigned long)ensMeasureCalls, (unsigned long)ensMeasureSlowestMs,
               (unsigned long)((millis() - ensMeasureWaitStart)),
               i2cLastSdaLevel, i2cLastSclLevel);
    perfI2cError();
    perfRecordMax(perfMaxEns160Ms,
                  (uint32_t)((micros() - perfEns160OnlyStartUs) / 1000UL));
    perfRecordMax(perfMaxI2cPathMs,
                  (uint32_t)((micros() - perfEnsStartUs) / 1000UL));
    return false;
  }

  // Raw hotplate data is not used by this sketch, but keep the existing raw
  // measurement call additive and non-blocking. It must never be allowed to
  // turn the bounded prediction measurement above back into an unbounded wait.
  perfI2cBegin(1, 2);
  uint32_t ensRawStartUs = micros();
  ens160.measureRaw(false);
  uint32_t ensRawElapsedMs = (uint32_t)((micros() - ensRawStartUs) / 1000UL);
  if (ensRawElapsedMs >= I2C_DIAG_SLOW_MS) {
    DBG_PRINTF("[ENS160 DIAG] measureRaw(false) time=%lums SDA=%d SCL=%d\n",
               (unsigned long)ensRawElapsedMs, i2cLastSdaLevel, i2cLastSclLevel);
  }

  // Read results — valid after the bounded prediction measurement above
  perfI2cBegin(1, 1);
  uint32_t ensGetStartUs = micros();
  tvoc    = (float)ens160.getTVOC();   // ppb
  eco2    = (float)ens160.geteCO2();   // ppm equivalent CO2
  aqi_out = (float)ens160.getAQI();    // 1–5 index
  uint32_t ensGetElapsedMs = (uint32_t)((micros() - ensGetStartUs) / 1000UL);
  if (ensGetElapsedMs >= I2C_DIAG_SLOW_MS) {
    DBG_PRINTF("[ENS160 DIAG] getTVOC/eCO2/AQI time=%lums SDA=%d SCL=%d\n",
               (unsigned long)ensGetElapsedMs, i2cLastSdaLevel, i2cLastSclLevel);
  }
  perfRecordMax(perfMaxEns160Ms,
                (uint32_t)((micros() - perfEns160OnlyStartUs) / 1000UL));

  perfRecordMax(perfMaxI2cPathMs,


                (uint32_t)((micros() - perfEnsStartUs) / 1000UL));


  return true;
}

// ============================================================================
// INMP441 I2S microphone — ESP-IDF 5.x standard driver/i2s_std.h
// The microphone now uses the ESP-IDF 5.x standard I2S driver. It is independent
// of the Arduino 3.x ADC driver, so there is no legacy-I2S/ADC ordering dependency.
// The older driver_ng conflict note is retained in the historical changelog above.
// ============================================================================

bool initINMP441()
{
  // Arduino-ESP32 3.x uses the ESP-IDF 5.x standard I2S driver.
  // Preserve the previous electrical configuration: master RX, 16kHz,
  // 32-bit stereo slots, GPIO25 BCLK, GPIO26 WS, GPIO33 data-in.
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = I2S_BUF_SAMPLES;

  esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &inmp441RxHandle);
  if (err != ESP_OK) {
    inmp441RxHandle = NULL;
    engMsgf("INMP441: i2s_new_channel failed (%d)", (int)err);
    return false;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_SCK_PIN,
      .ws = (gpio_num_t)I2S_WS_PIN,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)I2S_SD_PIN,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  err = i2s_channel_init_std_mode(inmp441RxHandle, &std_cfg);
  if (err != ESP_OK) {
    i2s_del_channel(inmp441RxHandle);
    inmp441RxHandle = NULL;
    engMsgf("INMP441: i2s_channel_init_std_mode failed (%d)", (int)err);
    return false;
  }

  err = i2s_channel_enable(inmp441RxHandle);
  if (err != ESP_OK) {
    i2s_del_channel(inmp441RxHandle);
    inmp441RxHandle = NULL;
    engMsgf("INMP441: i2s_channel_enable failed (%d)", (int)err);
    return false;
  }

  inmp441Initialized = true;
  engMsg("INMP441: I2S mic OK (Arduino-ESP32 3.x / ESP-IDF 5.x standard driver, stereo-slot capture)");
  return true;
}

// Read one buffer from INMP441, return RMS level converted to approximate dBFS.
// INMP441 is 24-bit left-justified in a 32-bit slot — shift right 8 to get
// the signed 24-bit value.  In core 2.0.17 we capture both slots and use the
// slot with the greater signal, so a legacy left/right-slot discrepancy cannot
// turn a working microphone into SENSOR_UNAVAILABLE.
float readINMP441_dB()
{
  int32_t samples[I2S_BUF_SAMPLES * 2];
  size_t  bytes_read = 0;
  inmpLastReadHadData = false;
  if (!inmp441Initialized) return SENSOR_UNAVAILABLE;

  esp_err_t err = i2s_channel_read(inmp441RxHandle,
                                   samples, sizeof(samples), &bytes_read,
                                   pdMS_TO_TICKS(50));
  if (err != ESP_OK) return SENSOR_UNAVAILABLE;

  int total = bytes_read / sizeof(int32_t);
  if (total < 2) return SENSOR_UNAVAILABLE;
  inmpLastReadHadData = true;

  double sumLeft = 0.0;
  double sumRight = 0.0;
  int leftCount = 0;
  int rightCount = 0;

  for (int i = 0; i + 1 < total; i += 2) {
    int32_t left  = samples[i]     >> 8;
    int32_t right = samples[i + 1] >> 8;
    sumLeft  += (double)left  * (double)left;
    sumRight += (double)right * (double)right;
    leftCount++;
    rightCount++;
  }

  if (leftCount == 0 || rightCount == 0) return SENSOR_UNAVAILABLE;

  const double rmsLeft  = sqrt(sumLeft  / leftCount);
  const double rmsRight = sqrt(sumRight / rightCount);
  const double rms = (rmsLeft >= rmsRight) ? rmsLeft : rmsRight;

  // Do not reject a valid very quiet microphone reading merely because the
  // RMS happens to be below one raw count.  Return a finite floor instead.
  if (rms <= 0.0) return SENSOR_UNAVAILABLE;

  float db = 20.0f * log10f((float)(rms / 8388608.0));
  if (!isfinite(db)) return SENSOR_UNAVAILABLE;
  if (db < -120.0f) db = -120.0f;
  return db;
}

// ============================================================================
// CO SENSOR — CLOSED LOOP heater control using A1 feedback
// ============================================================================

void setHeatDuty(byte duty)
{
  co_duty = duty;
  // v3.9.3: CO heater PWM shares the LEDC driver with the Core-1 buzzer.
  ledcWriteSafe(CO_PWM_PIN, duty);
}

void startMeasurementPhase()
{
  co_phase = 0; co_phase_start = millis();
  // Start at initial duty guess for 1.4V; closed loop in tickCO() will refine
  setHeatDuty(64);  // ~25% duty ≈ reasonable starting point for 1.4V target
  engMsg("CO: measure phase — closed-loop targeting 1.4V heater");
}

void startHeatingPhase()
{
  co_phase = 1; co_phase_start = millis();
  setHeatDuty(255);  // Full duty = full 5V heater
  engMsg("CO: heat phase (duty=255 → 5V heater)");
}

bool coSensorPresent()
{
  // GPIO36 (VP) floats near 0 when nothing is connected.
  // With MQ-7 divider circuit, it reads well above noise floor.
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(CO_ADC_PIN); delay(5); }
  return ((sum / 20) > 50);
}

float raw_to_CO_ppm(float adc_ema)
{
  if (adc_ema < 1.0f) return SENSOR_UNAVAILABLE;
  float adc_eq   = constrain(adc_ema * divider_scale, 1.0f, 4095.0f);
  float clean_eq = constrain(sensor_reading_clean_air * divider_scale, 1.0f, 4095.0f);
  sensor_base_resistance_kOhm =
    reference_resistor_kOhm * 4095.0f / clean_eq - reference_resistor_kOhm;
  if (sensor_reading_100_ppm_CO > 0.0f) {
    float c = constrain(sensor_reading_100_ppm_CO * divider_scale, 1.0f, 4095.0f);
    sensor_100ppm_CO_resistance_kOhm =
      reference_resistor_kOhm * 4095.0f / c - reference_resistor_kOhm;
  } else {
    sensor_100ppm_CO_resistance_kOhm = sensor_base_resistance_kOhm * 0.5f;
  }
  if (sensor_base_resistance_kOhm <= 0.0f || sensor_100ppm_CO_resistance_kOhm <= 0.0f)
    return SENSOR_UNAVAILABLE;
  float sensor_R = reference_resistor_kOhm * 4095.0f / adc_eq - reference_resistor_kOhm;
  if (sensor_R <= 0.0f) return 0.0f;
  float ppm = 100.0f * (expf(sensor_100ppm_CO_resistance_kOhm / sensor_R) - 1.648f);
  return (ppm < 0.0f) ? 0.0f : ppm;
}

void tickCO()
{
  unsigned long elapsed = millis() - co_phase_start;

  // Read A1 heater feedback (via divider, so multiply by scale to get actual V)
  float a1_adc_v = adcToVolts(averageAnalogRead(CO_A1_PIN));  // voltage at ADC pin
  float heater_v = a1_adc_v * divider_scale;                  // actual heater voltage
  lastCoHeaterV = heater_v;

  // Closed-loop duty adjustment toward target
  // Target is in ADC-pin volts (after divider).
  // Simple proportional: ±1 duty step per tick when outside tolerance band.
  float target = (co_phase == 1) ? CO_HEAT_TARGET_V : CO_MEAS_TARGET_V;
  float error  = target - a1_adc_v;
  if (fabsf(error) > CO_CLOSED_LOOP_TOL) {
    int new_duty = (int)co_duty + (error > 0 ? 1 : -1);
    new_duty = constrain(new_duty, 0, 255);
    setHeatDuty((byte)new_duty);
  }

  // Watchdog — phase stuck beyond CO_PHASE_MAX_MS
  if (elapsed > CO_PHASE_MAX_MS) {
    engMsgf("CO WATCHDOG: phase %d stuck >%lus heater=%.2fV",
            co_phase, CO_PHASE_MAX_MS/1000, heater_v);
    (co_phase == 1) ? startMeasurementPhase() : startHeatingPhase();
    if (dataMutexTake(5) == pdTRUE) {
      sd.co_fault = true; sd.status_flags |= STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    return;
  }

  // Phase transitions
  if (co_phase == 1 && elapsed > 60000UL) {
    startMeasurementPhase();
    if (dataMutexTake(5) == pdTRUE) {
      sd.co_fault = false; sd.status_flags &= ~STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    return;
  }
  if (co_phase == 0 && elapsed > 90000UL) {
    float ppm = raw_to_CO_ppm(sens_val);
    if (ppm >= 0.0f) last_CO_ppm = ppm;
    sens_val_last = sens_val;
    engMsgf("CO: cycle done ppm=%.1f raw=%.0f heater=%.2fV duty=%d",
            last_CO_ppm, sens_val, heater_v, co_duty);
    if (dataMutexTake(5) == pdTRUE) {
      sd.co_ppm = last_CO_ppm; sd.co_raw = sens_val; sd.co_phase = co_phase;
      sd.co_heater_v = heater_v;
      sd.co_fault = (ppm == SENSOR_UNAVAILABLE);
      if (ppm == SENSOR_UNAVAILABLE) sd.status_flags |= STATUS_CO_FAULT;
      else                           sd.status_flags &= ~STATUS_CO_FAULT;
      xSemaphoreGive(dataMutex);
    }
    startHeatingPhase();
    return;
  }

  // Continuous CO sense ADC read + exponential moving average (α=0.3)
  float v = (float)analogRead(CO_ADC_PIN);
  float maxADC = 4095.0f * divider_ratio;
  if (v >= 10.0f && v <= maxADC * 1.05f)
    sens_val = 0.7f * sens_val + 0.3f * v;

  // Update shared struct — brief mutex hold
  if (dataMutexTake(5) == pdTRUE) {
    sd.co_raw = sens_val; sd.co_phase = co_phase;
    sd.co_heater_v = heater_v;
    xSemaphoreGive(dataMutex);
  }

  if (DEBUGON)
    DBG_PRINTF("CO phase=%d t=%lus ema=%.0f ppm=%.1f a1_v=%.3f heater_v=%.3f duty=%d\n",
                  co_phase, elapsed/1000, sens_val, last_CO_ppm, a1_adc_v, heater_v, co_duty);
}

// ============================================================================
// ─── I2C FAILURE ISOLATION / BUS RECOVERY — v3.9.23 ─────────────────────────
// This layer existed in v3.7 but was accidentally lost while the v3.9.x
// network diagnostics were merged. A mechanically intermittent I2C device can
// leave SDA low or stop ACKing; entering the full sensor-library transaction
// in that state can turn a small sensor fault into a multi-second Core-0 stall.
// Preflight + bounded Wire timeout + conservative 9-clock recovery keep that
// fault isolated from the WiFi/Blynk side of the firmware.
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_TIMEOUT_MS 50
#define I2C_RECOVERY_MIN_INTERVAL_MS 2000UL
#define ENS_I2C_REINIT_FAILS 2
#define BMI_I2C_REINIT_FAILS 3
unsigned long lastI2CRecovery = 0;
unsigned long lastI2CFaultMsg = 0;
volatile uint32_t i2cProbeFailCount = 0;
volatile uint32_t i2cBusRecoveryCount = 0;
// i2cLastSdaLevel/i2cLastSclLevel moved earlier in the file (before readENS())
// — see the v3.9.42 comment there. Not redeclared here.

// v3.9.32 I2C/IMU diagnostics. These are deliberately lightweight and
// rate-limited: they record the exact I2C stage that was slow/failing without
// changing the normal sensor scheduling or adding another blocking path.
volatile uint8_t i2cLastWireError = 0;
volatile uint32_t i2cDiagnosticCount = 0;
unsigned long lastI2CDiagnosticMsg = 0;
// I2C_DIAG_SLOW_MS is defined near the top of the file for Arduino
// auto-generated prototype compatibility; do not redeclare it here.

static void i2cCaptureLineState()
{
  i2cLastSdaLevel = (uint8_t)digitalRead(I2C_SDA_PIN);
  i2cLastSclLevel = (uint8_t)digitalRead(I2C_SCL_PIN);
}

bool i2cDevicePresent(uint8_t address)
{
  i2cCaptureLineState();
  uint32_t startUs = micros();
  Wire.beginTransmission(address);
  uint8_t err = Wire.endTransmission();
  uint32_t elapsedUs = micros() - startUs;
  i2cLastWireError = err;
  i2cCaptureLineState();

  if (err != 0) {
    ++i2cProbeFailCount;
    if ((elapsedUs / 1000UL) >= I2C_DIAG_SLOW_MS &&
        millis() - lastI2CDiagnosticMsg >= 1000UL) {
      lastI2CDiagnosticMsg = millis();
      DBG_PRINTF("[I2C DIAG] probe 0x%02X took %lums err=%u SDA=%d SCL=%d\n",
                 address, (unsigned long)(elapsedUs / 1000UL), err,
                 i2cLastSdaLevel, i2cLastSclLevel);
    }
    return false;
  }
  return true;
}

void i2cBusRecover()
{
  unsigned long now = millis();
  if (now - lastI2CRecovery < I2C_RECOVERY_MIN_INTERVAL_MS) return;
  lastI2CRecovery = now;

  // First detach the Arduino TwoWire peripheral so a previously aborted
  // transaction cannot leave its internal state attached to the bus.
  Wire.end();
  delayMicroseconds(20);

  // Release SDA and manually generate up to 9 SCL clocks. This is the standard
  // I2C slave-unlock sequence for a device that was interrupted mid-byte.
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < 9; ++i) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // Generate an explicit STOP while both lines are released.
  pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(5);

  // Reattach TwoWire and restore the bounded transaction timeout.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  delayMicroseconds(20);
  i2cCaptureLineState();
  ++i2cBusRecoveryCount;

  DBG_PRINTF("[I2C] bus recovery attempted SDA=%d SCL=%d count=%lu\n",
             i2cLastSdaLevel, i2cLastSclLevel,
             (unsigned long)i2cBusRecoveryCount);

  if (i2cLastSdaLevel == LOW || i2cLastSclLevel == LOW) {
    DBG_PRINTF("[I2C] BUS STILL HELD LOW after recovery — external power/device fault likely\n");
  }
}
bool i2cSensorPresent(uint8_t address, const char* name)
{
  if (i2cDevicePresent(address)) return true;
  DBG_PRINTF("[I2C] %s (0x%02X) not responding SDA=%d SCL=%d\n",
             name, address, i2cLastSdaLevel, i2cLastSclLevel);
  sensorHealthMsg(name, "I2C device not responding", lastI2CFaultMsg);
  i2cBusRecover();
  return false;
}

// GY-BMI160 6-DOF IMU — minimal I2C register-level driver
// [SUPERSEDED v2.4] JY-901 (UART, pre-fused Roll/Pitch/Yaw) has been removed
// entirely. BMI160 is now the ONLY IMU — its raw gyro/accel feed both
// inertialNav() (EKF dead-reckoning) AND, via a complementary filter inside
// inertialNav() itself, the Roll/Pitch/Yaw/temp values sent to V8-V11.
// feeding V8-V11 exactly as before.
// ============================================================================

bool bmi160WriteReg(uint8_t reg, uint8_t val)
{
  if (!i2cDevicePresent(bmi160ActiveAddr)) return false;
  uint32_t startUs = micros();
  Wire.beginTransmission(bmi160ActiveAddr);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  uint32_t elapsedUs = micros() - startUs;
  i2cLastWireError = err;
  if (err != 0) perfI2cError();

  if ((elapsedUs / 1000UL) >= I2C_DIAG_SLOW_MS &&
      millis() - lastI2CDiagnosticMsg >= 1000UL) {
    lastI2CDiagnosticMsg = millis();
    DBG_PRINTF("[BMI160 DIAG] WRITE reg=0x%02X val=0x%02X addr=0x%02X took=%lums err=%u SDA=%d SCL=%d\n",
               reg, val, bmi160ActiveAddr, (unsigned long)(elapsedUs / 1000UL),
               err, i2cLastSdaLevel, i2cLastSclLevel);
  }
  return (err == 0);
}

bool bmi160ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
  if (!i2cDevicePresent(bmi160ActiveAddr)) return false;

  uint32_t startUs = micros();
  Wire.beginTransmission(bmi160ActiveAddr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);   // repeated start
  i2cLastWireError = err;
  if (err != 0) {
    perfI2cError();
    i2cCaptureLineState();
    DBG_PRINTF("[BMI160 DIAG] ADDR/REG reg=0x%02X addr=0x%02X endTx err=%u SDA=%d SCL=%d\n",
               reg, bmi160ActiveAddr, err, i2cLastSdaLevel, i2cLastSclLevel);
    return false;
  }

  uint8_t got = Wire.requestFrom((int)bmi160ActiveAddr, (int)len);
  if (got != len) {
    i2cLastWireError = 0;
    perfI2cError();
    i2cCaptureLineState();
    DBG_PRINTF("[BMI160 DIAG] READ reg=0x%02X addr=0x%02X requested=%u got=%u SDA=%d SCL=%d\n",
               reg, bmi160ActiveAddr, len, got, i2cLastSdaLevel, i2cLastSclLevel);
    return false;
  }

  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();

  uint32_t elapsedUs = micros() - startUs;
  if ((elapsedUs / 1000UL) >= I2C_DIAG_SLOW_MS &&
      millis() - lastI2CDiagnosticMsg >= 1000UL) {
    lastI2CDiagnosticMsg = millis();
    DBG_PRINTF("[BMI160 DIAG] READ reg=0x%02X len=%u addr=0x%02X took=%lums SDA=%d SCL=%d\n",
               reg, len, bmi160ActiveAddr, (unsigned long)(elapsedUs / 1000UL),
               i2cLastSdaLevel, i2cLastSclLevel);
  }
  return true;
}

bool bmi160ProbeAndSelectAddress()
{
  // The previous firmware used 0x69 and the IMU worked before moving to
  // Arduino-ESP32 2.0.17. Keep 0x69 as the first choice, but explicitly test
  // 0x68 as well so an address/SA0 issue cannot be mistaken for an I2C hang.
  const uint8_t candidates[2] = {0x69, 0x68};

  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t addr = candidates[i];
    uint32_t startUs = micros();
    bool present = i2cDevicePresent(addr);
    uint32_t elapsedMs = (micros() - startUs) / 1000UL;

    DBG_PRINTF("[BMI160 DIAG] probe addr=0x%02X present=%d time=%lums err=%u SDA=%d SCL=%d\n",
               addr, present ? 1 : 0, (unsigned long)elapsedMs,
               i2cLastWireError, i2cLastSdaLevel, i2cLastSclLevel);

    if (!present) continue;

    uint8_t chipId = 0;
    startUs = micros();
    bool ok = bmi160ReadRegs(BMI160_REG_CHIPID, &chipId, 1);
    elapsedMs = (micros() - startUs) / 1000UL;

    DBG_PRINTF("[BMI160 DIAG] chipID addr=0x%02X ok=%d value=0x%02X time=%lums err=%u SDA=%d SCL=%d\n",
               addr, ok ? 1 : 0, chipId, (unsigned long)elapsedMs,
               i2cLastWireError, i2cLastSdaLevel, i2cLastSclLevel);

    if (ok && chipId == BMI160_CHIPID_EXPECTED) {
      bmi160ActiveAddr = addr;
      DBG_PRINTF("[BMI160 DIAG] SELECTED address 0x%02X (chip ID 0x%02X)\n",
                 bmi160ActiveAddr, chipId);
      return true;
    }
  }

  DBG_PRINTF("[BMI160 DIAG] NO VALID BMI160 at 0x69 or 0x68 — expected chip ID 0x%02X\n",
             BMI160_CHIPID_EXPECTED);
  return false;
}

bool initBMI160()
{
  uint8_t chipId = 0;

  uint32_t initStartUs = micros();
  if (!bmi160ProbeAndSelectAddress()) {
    engMsgf("BMI160: no valid chip ID at 0x69/0x68 (expected 0x%02X)",
            BMI160_CHIPID_EXPECTED);
    return false;
  }

  if (!bmi160ReadRegs(BMI160_REG_CHIPID, &chipId, 1) ||
      chipId != BMI160_CHIPID_EXPECTED) {
    engMsgf("BMI160: chip ID 0x%02X (expected 0x%02X) — check wiring/address",
            chipId, BMI160_CHIPID_EXPECTED);
    return false;
  }

  DBG_PRINTF("[BMI160 DIAG] init preamble completed in %lums at addr=0x%02X\n",
             (unsigned long)((micros() - initStartUs) / 1000UL), bmi160ActiveAddr);

  bmi160WriteReg(BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
  delay(50);
  // ODR=100Hz (0x08) | bandwidth=normal (0x02<<4=0x20) = 0x28, for both acc & gyro
  bmi160WriteReg(BMI160_REG_ACC_CONF, 0x28);
  bmi160WriteReg(BMI160_REG_ACC_RANGE, 0x05);   // ±4g
  bmi160WriteReg(BMI160_REG_GYR_CONF, 0x28);
  bmi160WriteReg(BMI160_REG_GYR_RANGE, 0x02);   // ±500 dps
  bmi160WriteReg(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(50);
  bmi160WriteReg(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(100);
  engMsgf("BMI160: init OK (±4g, ±500dps, 100Hz, I2C 0x%02X)", bmi160ActiveAddr);
  bmi_i2c_fail_streak = 0;
  return true;
}

// Returns physical units: gx/gy/gz in dps, ax/ay/az in g.
bool bmi160ReadRaw(float& gx, float& gy, float& gz, float& ax, float& ay, float& az)
{
  uint32_t perfBmi160RawStartUs = micros();
  perfI2cBegin(3, 1);
  uint8_t buf[12];
  if (!bmi160ReadRegs(BMI160_REG_DATA, buf, 12)) {
    perfI2cError();
    if (bmi_i2c_fail_streak < 255) bmi_i2c_fail_streak++;
    if (bmi_i2c_fail_streak >= BMI_I2C_REINIT_FAILS) {
      bmi160Ready = false;
      DBG_PRINTF("[I2C RECOVERY] BMI160 marked not-ready after %u consecutive failed raw reads — reinitializing\n",
                 (unsigned)bmi_i2c_fail_streak);
      i2cBusRecover();
    }
    perfRecordMax(perfMaxBmi160Ms,
                  (uint32_t)((micros() - perfBmi160RawStartUs) / 1000UL));
    return false;
  }
  bmi_i2c_fail_streak = 0;
  int16_t rgx = (int16_t)((buf[1]<<8)|buf[0]);
  int16_t rgy = (int16_t)((buf[3]<<8)|buf[2]);
  int16_t rgz = (int16_t)((buf[5]<<8)|buf[4]);
  int16_t rax = (int16_t)((buf[7]<<8)|buf[6]);
  int16_t ray = (int16_t)((buf[9]<<8)|buf[8]);
  int16_t raz = (int16_t)((buf[11]<<8)|buf[10]);
  gx = rgx / 32768.0f * BMI160_GYR_RANGE_DPS;
  gy = rgy / 32768.0f * BMI160_GYR_RANGE_DPS;
  gz = rgz / 32768.0f * BMI160_GYR_RANGE_DPS;
  ax = rax / 32768.0f * BMI160_ACC_RANGE_G;
  ay = ray / 32768.0f * BMI160_ACC_RANGE_G;
  az = raz / 32768.0f * BMI160_ACC_RANGE_G;

  if (DEBUGON) {
    static unsigned long lastGPSDebug = 0;
    if (millis() - lastGPSDebug >= 5000) {
      lastGPSDebug = millis();
      DBG_PRINTF("[IMU DEBUG] gx=%.1f gy=%.1f gz=%.1f | ax=%.1f ay=%.1f az=%.1f \n",
                      gx,
                      gy,
                      gz,
                      ax,
                      ay,
                      az);
    }
  }

  perfRecordMax(perfMaxBmi160Ms,
                (uint32_t)((micros() - perfBmi160RawStartUs) / 1000UL));
  return true;
}

// ============================================================================
// INERTIAL NAVIGATION — GPS + BMI160 4-state EKF (v2.3)
// See the v2.3 CHANGES header comment block at the top of this file for the
// full design rationale (why velocity isn't double-integrated from the
// accelerometer, why heading uses gyro dead-reckoning + GPS-course
// correction only above INAV_MIN_COURSE_SPEED_MPS, and why gyro bias is
// calibrated via ZUPT instead of being a filter state).
// ============================================================================

float inavWrapAngle(float a)
{
  while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
  while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
  return a;
}

// General 4x4 matrix inverse via Gauss-Jordan with partial pivoting.
// Only used by the "full" GPS update (position+speed+heading, H=Identity),
// which fires at most a few times per second — cost is negligible.
bool mat4Inverse(const float M[4][4], float Inv[4][4])
{
  float A[4][8];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) A[i][j] = M[i][j];
    for (int j = 0; j < 4; j++) A[i][4+j] = (i==j) ? 1.0f : 0.0f;
  }
  for (int col = 0; col < 4; col++) {
    int piv = col;
    float maxAbs = fabsf(A[col][col]);
    for (int r = col+1; r < 4; r++) {
      if (fabsf(A[r][col]) > maxAbs) { maxAbs = fabsf(A[r][col]); piv = r; }
    }
    if (maxAbs < 1e-9f) return false;   // singular — caller should skip this update
    if (piv != col) {
      for (int k = 0; k < 8; k++) { float tmp=A[col][k]; A[col][k]=A[piv][k]; A[piv][k]=tmp; }
    }
    float d = A[col][col];
    for (int k = 0; k < 8; k++) A[col][k] /= d;
    for (int r = 0; r < 4; r++) {
      if (r == col) continue;
      float f = A[r][col];
      if (f == 0.0f) continue;
      for (int k = 0; k < 8; k++) A[r][k] -= f*A[col][k];
    }
  }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      Inv[i][j] = A[i][4+j];
  return true;
}

// Prediction step — nonlinear unicycle model, linearised via Jacobian F.
// dt in seconds. gyroZ_dps is the RAW BMI160 gyro Z reading; bias is
// subtracted here using the ZUPT-calibrated inavGyroBiasDps.
void inavPredict(float dt, float gyroZ_dps)
{
  float yawRate = (gyroZ_dps - inavGyroBiasDps) * DEG_TO_RAD;  // rad/s
  float psi = inavX[3];
  float v   = inavX[2];
  float s = sinf(psi), c = cosf(psi);

  inavX[0] += v * s * dt;
  inavX[1] += v * c * dt;
  inavX[3]  = inavWrapAngle(psi + yawRate * dt);
  // v (inavX[2]) held constant here — corrected only by GPS or ZUPT.

  float F[4][4] = {
    {1, 0,  s*dt,   v*c*dt},
    {0, 1,  c*dt,  -v*s*dt},
    {0, 0,  1,      0     },
    {0, 0,  0,      1     }
  };

  float FP[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += F[i][k]*inavP[k][j];
    FP[i][j]=acc;
  }
  float FPFt[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += FP[i][k]*F[j][k];  // F^T[k][j]=F[j][k]
    FPFt[i][j]=acc;
  }
  static const float Q[4] = {INAV_Q_POS, INAV_Q_POS, INAV_Q_SPEED, INAV_Q_HEADING};
  for (int i=0;i<4;i++) for (int j=0;j<4;j++)
    inavP[i][j] = FPFt[i][j] + ((i==j) ? Q[i] : 0.0f);
}

// GPS update — position only (H = [[1,0,0,0],[0,1,0,0]]). Hand-expanded
// 2-row special case instead of a generic H matrix — cheaper and clearer.
void inavKalmanUpdatePos(float zE, float zN, float rPos)
{
  float yE = zE - inavX[0];
  float yN = zN - inavX[1];
  float S00 = inavP[0][0] + rPos, S01 = inavP[0][1];
  float S10 = inavP[1][0],        S11 = inavP[1][1] + rPos;
  float det = S00*S11 - S01*S10;
  if (fabsf(det) < 1e-9f) return;   // singular — skip this update
  float invDet = 1.0f/det;
  float Si00 =  S11*invDet, Si01 = -S01*invDet;
  float Si10 = -S10*invDet, Si11 =  S00*invDet;

  float K[4][2];
  for (int i=0;i<4;i++) {
    float PHt0 = inavP[i][0], PHt1 = inavP[i][1];
    K[i][0] = PHt0*Si00 + PHt1*Si10;
    K[i][1] = PHt0*Si01 + PHt1*Si11;
  }
  for (int i=0;i<4;i++) inavX[i] += K[i][0]*yE + K[i][1]*yN;
  inavX[3] = inavWrapAngle(inavX[3]);

  float HP0[4], HP1[4];
  for (int j=0;j<4;j++) { HP0[j]=inavP[0][j]; HP1[j]=inavP[1][j]; }
  for (int i=0;i<4;i++) for (int j=0;j<4;j++)
    inavP[i][j] -= K[i][0]*HP0[j] + K[i][1]*HP1[j];
}

// GPS update — full observation (H = Identity4), used only when GPS speed
// and course are both valid AND speed exceeds INAV_MIN_COURSE_SPEED_MPS.
void inavKalmanUpdateFull(float zE, float zN, float zSpeed, float zHeadingRad,
                           float rPos, float rSpeed, float rHeading)
{
  float y[4] = { zE - inavX[0], zN - inavX[1], zSpeed - inavX[2],
                 inavWrapAngle(zHeadingRad - inavX[3]) };
  float S[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) S[i][j] = inavP[i][j];
  S[0][0]+=rPos; S[1][1]+=rPos; S[2][2]+=rSpeed; S[3][3]+=rHeading;

  float Sinv[4][4];
  if (!mat4Inverse(S, Sinv)) return;   // singular — skip this update

  float K[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += inavP[i][k]*Sinv[k][j];
    K[i][j]=acc;
  }
  for (int i=0;i<4;i++) {
    float dx=0; for (int j=0;j<4;j++) dx += K[i][j]*y[j];
    inavX[i] += dx;
  }
  inavX[3] = inavWrapAngle(inavX[3]);

  float KP[4][4];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
    float acc=0; for (int k=0;k<4;k++) acc += K[i][k]*inavP[k][j];
    KP[i][j]=acc;
  }
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) inavP[i][j] -= KP[i][j];
}

// Zero-velocity pseudo-measurement — 1-D Kalman update pulling state[2]
// (speed) toward zero. Factored out so both inavZUPT() (below) and the
// IMU-variance-confirmed-stationary path in inertialNav() can apply the same
// correction without duplicating the Kalman math.
void inavApplyZeroVelocity()
{
  float y = 0.0f - inavX[2];
  float S = inavP[2][2] + INAV_ZUPT_R_V;
  if (S <= 1e-9f) return;
  float K[4];
  for (int i=0;i<4;i++) K[i] = inavP[i][2]/S;
  for (int i=0;i<4;i++) inavX[i] += K[i]*y;
  float P2row[4]; for (int j=0;j<4;j++) P2row[j]=inavP[2][j];
  for (int i=0;i<4;i++) for (int j=0;j<4;j++) inavP[i][j] -= K[i]*P2row[j];
}

// Zero-velocity update (ZUPT) — the standard robust technique for keeping a
// dead-reckoning filter from drifting while stationary, and for calibrating
// gyro bias without needing it to be a filter state (see header rationale).
// Reacts to a single instantaneous sample being close to "at rest".
void inavZUPT(float ax_g, float ay_g, float az_g, float gyroZ_dps)
{
  float accelMag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  bool stationary = (fabsf(accelMag - 1.0f) < INAV_ZUPT_ACCEL_TOL_G) &&
                     (fabsf(gyroZ_dps) < INAV_ZUPT_GYRO_TOL_DPS);
  unsigned long now = millis();
  if (!stationary) { zuptStartMillis = 0; return; }
  if (zuptStartMillis == 0) { zuptStartMillis = now; return; }
  if (now - zuptStartMillis < INAV_ZUPT_DURATION_MS) return;

  // Gyro bias: slow EMA toward the current (true-rate-is-zero) reading.
  inavGyroBiasDps = 0.98f*inavGyroBiasDps + 0.02f*gyroZ_dps;

  inavApplyZeroVelocity();
}

// Statistical (variance-based) stillness detector — complements ZUPT's
// instantaneous-threshold check above. A single accel/gyro sample near "at
// rest" can still coincide with genuine slow motion, and conversely a noisy
// single sample can look like motion while the device is actually still.
// Tracking the rolling VARIANCE of accel- and gyro-magnitude over a sustained
// window is a much more reliable "is this actually moving, or just sensor
// noise?" test than any one-shot threshold: real motion keeps the variance
// elevated, while noise around a fixed true value does not sustain it.
//
// This feeds imuConfirmedStationary, which inertialNav() uses to decide
// whether a small nonzero GPS-reported speed (commonly 0.3-1.0 m/s on cheap
// modules like the ATGM336H, from multipath/doppler jitter even at a full
// stop) should be trusted as real motion or treated as GPS speed noise.
void imuVarianceCheck(float ax_g, float ay_g, float az_g,
                       float gx_dps, float gy_dps, float gz_dps)
{
  float accelMag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float gyroMag  = sqrtf(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);

  if (!imuVarEmaInited) {
    imuAccelMagEma = accelMag; imuGyroMagEma = gyroMag;
    imuAccelVarEma = 0.0f;     imuGyroVarEma  = 0.0f;
    imuVarEmaInited = true;
  } else {
    float da = accelMag - imuAccelMagEma;
    float dg = gyroMag  - imuGyroMagEma;
    imuAccelMagEma += INAV_IMU_VAR_ALPHA * da;
    imuGyroMagEma  += INAV_IMU_VAR_ALPHA * dg;
    // EMA of squared deviation from the EMA mean approximates rolling variance.
    imuAccelVarEma += INAV_IMU_VAR_ALPHA * (da*da - imuAccelVarEma);
    imuGyroVarEma  += INAV_IMU_VAR_ALPHA * (dg*dg - imuGyroVarEma);
  }

  bool lowVariance = (imuAccelVarEma < INAV_IMU_ACCEL_VAR_TOL_G2) &&
                      (imuGyroVarEma  < INAV_IMU_GYRO_VAR_TOL_DPS2);

  unsigned long now = millis();
  if (!lowVariance) {
    imuStillStartMillis = 0;
    imuConfirmedStationary = false;
    return;
  }
  if (imuStillStartMillis == 0) { imuStillStartMillis = now; imuConfirmedStationary = false; return; }
  imuConfirmedStationary = (now - imuStillStartMillis >= INAV_IMU_STILL_DURATION_MS);
}

// Master entry point — called once per sensorTask loop iteration. Internally
// rate-limits its own prediction step to inav_update_hz; GPS updates run
// whenever TinyGPS++ reports a fresh location this cycle (no rate limiting
// needed there — GPS naturally updates at its own, much lower, rate).
void inertialNav()
{
  uint32_t perfInavStartUs = micros();

  static bool          originSet            = false;
  static double        lat0 = 0, lon0 = 0;
  static float         cosLat0              = 1.0f;
  static int           lastInavState        = -1;
  static unsigned long lastGoodFixMillis    = 0;
  static unsigned long gpsFixIntervalMsLoc  = 1000;  // adaptive nominal GPS interval
  static unsigned long lastPredictMillis    = 0;
  static unsigned long lastBmiRetryMillis   = 0;
  static bool          firstBmiAttempt      = true;   // bypass the 10s gate on the very first call

  if (!bmi160Ready) {
    if (firstBmiAttempt || millis() - lastBmiRetryMillis > 10000) {
      firstBmiAttempt = false;
      lastBmiRetryMillis = millis();
      bmi160Ready = initBMI160();
    }
    if (!bmi160Ready) perfRecordMax(perfMaxI2cPathMs,
               (uint32_t)((micros() - perfInavStartUs) / 1000UL));
 return;   // device fully absent — nothing to do this cycle
  }

  float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;
  bool bmiOk = bmi160ReadRaw(gx,gy,gz,ax,ay,az);
  if (bmiOk) lastBmiOkMillis = millis();
  // Unlike a hard I2C absence, a single failed read no longer aborts the whole
  // cycle — GPS/EKF processing below still runs (gz=0 is a safe one-cycle
  // fallback for the predict step). "Stuck" means sustained failure.
  bool imuStuck = (lastBmiOkMillis > 0 && millis() - lastBmiOkMillis > IMU_TIMEOUT_MS);
  // Don't let a stale "confirmed stationary" verdict (from before the IMU
  // stopped producing good reads) keep suppressing GPS speed corrections.
  if (imuStuck) imuConfirmedStationary = false;

  if (bmiOk) {
    const unsigned long nowImuYaw = millis();
    if (lastImuYawMillis != 0) {
      const float dtYaw = constrain((nowImuYaw - lastImuYawMillis) / 1000.0f, 0.0f, 0.5f);
      imuYawDeg += (gz - inavGyroBiasDps) * dtYaw;
      while (imuYawDeg >= 360.0f) imuYawDeg -= 360.0f;
      while (imuYawDeg < 0.0f)   imuYawDeg += 360.0f;
    }
    lastImuYawMillis = nowImuYaw;

    inavZUPT(ax,ay,az,gz);
    imuVarianceCheck(ax,ay,az,gx,gy,gz);

    // ── Roll/Pitch complementary filter (v2.4, replaces JY-901 fusion) ──────
    // Gravity-vector accel angle blended with gyro-integrated angle at 98/2.
    // Gravity gives an absolute, non-drifting reference for these two axes,
    // so a simple complementary filter (not a full AHRS) is sufficient here.
    unsigned long nowFusion = millis();
    float dtFusion = (lastFusionMillis == 0) ? 0.0f
                    : constrain((nowFusion - lastFusionMillis) / 1000.0f, 0.0f, 0.5f);
    lastFusionMillis = nowFusion;

    float rollAcc  = atan2f(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
    const float ALPHA = 0.98f;
    angle[0] = ALPHA * (angle[0] + gx*dtFusion) + (1.0f-ALPHA) * rollAcc;   // Roll
    angle[1] = ALPHA * (angle[1] + gy*dtFusion) + (1.0f-ALPHA) * pitchAcc;  // Pitch
    // angle[2] (Yaw) is set further below, from the EKF heading state —
    // BMI160 has no magnetometer, so free-integrating yaw here would drift
    // unboundedly; the EKF heading is already GPS-corrected, so we reuse it.

    // BMI160 die temperature (registers 0x20/0x21) — informational, feeds V11
    uint8_t tbuf[2];
    if (bmi160ReadRegs(BMI160_REG_TEMP, tbuf, 2)) {
      int16_t rawT = (int16_t)((tbuf[1]<<8) | tbuf[0]);
      imuT = (rawT == (int16_t)0x8000) ? SENSOR_UNAVAILABLE : (23.0f + rawT/512.0f);
    }
  }

  // v3.5: feedGPS/raw-GPS processing may already have read lat()/lng(),
  // which clears TinyGPS++'s location.isUpdated() flag. Use the snapshot
  // captured immediately after feedGPS() so INAV never misses a fresh fix.
  bool newFix = gpsLocationUpdatedThisCycle;
  int state;

  if (newFix && gps.location.isValid()) {
    double glat = gps.location.lat();
    double glon = gps.location.lng();

    if (!originSet) {
      lat0 = glat; lon0 = glon; cosLat0 = cosf((float)lat0 * DEG_TO_RAD);
      inavX[0]=0; inavX[1]=0; inavX[2]=0; inavX[3]=0;
      memset(inavP, 0, sizeof(inavP));
      inavP[0][0]=inavP[1][1]=100.0f;      // ~10m initial position uncertainty
      inavP[2][2]=4.0f;                     // ~2 m/s initial speed uncertainty
      inavP[3][3]=(float)(M_PI*M_PI);       // heading fully unknown initially
      originSet = true;
      engMsg("INAV: origin set from first GPS fix");
    }

    if (lastGoodFixMillis > 0) {
      unsigned long dtFix = millis() - lastGoodFixMillis;
      if (dtFix > 50 && dtFix < 10000)
        gpsFixIntervalMsLoc = (unsigned long)(0.7f*gpsFixIntervalMsLoc + 0.3f*dtFix);
    }
    lastGoodFixMillis = millis();

    float zE = (float)((glon - lon0) * DEG_TO_RAD * cosLat0 * EARTH_RADIUS_M);
    float zN = (float)((glat - lat0) * DEG_TO_RAD * EARTH_RADIUS_M);

    bool haveSpeedCourse = gps.speed.isValid() && gps.course.isValid();
    float gpsSpeedMps = haveSpeedCourse ? (float)gps.speed.mps() : 0.0f;

    // imuConfirmedStationary distrusts a nonzero GPS speed/course reading
    // when the IMU's variance-based check confirms the device isn't actually
    // moving — a raised GPS speed here is then GPS noise, not real motion.
    if (haveSpeedCourse && gpsSpeedMps > INAV_MIN_COURSE_SPEED_MPS &&
        !imuConfirmedStationary) {
      float zHeading = (float)gps.course.deg() * DEG_TO_RAD;
      inavKalmanUpdateFull(zE, zN, gpsSpeedMps, zHeading,
                            INAV_R_POS, INAV_R_SPEED, INAV_R_HEADING);
    } else {
      inavKalmanUpdatePos(zE, zN, INAV_R_POS);
      // IMU statistically confirms stillness — actively correct the speed
      // state toward zero instead of leaving it pulled up by GPS noise.
      if (imuConfirmedStationary) inavApplyZeroVelocity();
    }
    state = INAV_STATE_GPS_FIX;
  }
  else if (!originSet) {
    state = INAV_STATE_NO_FIX_YET;
  }
  else {
    unsigned long sinceFix = millis() - lastGoodFixMillis;
    unsigned long missedEq = sinceFix / gpsFixIntervalMsLoc;
    unsigned long maxLossMs = (unsigned long)inav_max_loss_minutes * 60000UL;
    if      (sinceFix >= maxLossMs)                    state = INAV_STATE_LOST;
    else if ((int)missedEq >= inav_stale_after_misses) state = INAV_STATE_IMU_STALE;
    else                                                state = INAV_STATE_IMU_RECENT;
  }

  // Prediction — rate-limited to the user-configured inav_update_hz. Frozen
  // entirely once LOST, exactly as requested: no further advancement of the
  // dead-reckoned position after the max-loss timeout.
  if (originSet && state != INAV_STATE_LOST) {
    unsigned long now = millis();
    unsigned long periodMs = (unsigned long)(1000.0f / max(1.0f, inav_update_hz));
    if (lastPredictMillis == 0) lastPredictMillis = now;
    if (now - lastPredictMillis >= periodMs) {
      float dt = (now - lastPredictMillis) / 1000.0f;
      inavPredict(dt, gz);
      lastPredictMillis = now;
    }
  }

  if (state != lastInavState) {
    switch (state) {
      case INAV_STATE_NO_FIX_YET: engMsg("INAV: waiting for first GPS fix"); break;
      case INAV_STATE_GPS_FIX:    engMsg("INAV: position acquired by GPS"); break;
      case INAV_STATE_IMU_RECENT: engMsg("INAV: GPS gap — interpolating from recent GPS+IMU"); break;
      case INAV_STATE_IMU_STALE:  engMsgf("INAV: GPS lost >=%d fixes — interpolating (stale baseline)",
                                           inav_stale_after_misses); break;
      case INAV_STATE_LOST:       engMsgf("INAV: GPS lost >%d min — interpolation stopped, holding position",
                                           inav_max_loss_minutes); break;
    }
    lastInavState = state;
  }

  double outLat = lat0, outLon = lon0;
  float  outAlt = SENSOR_UNAVAILABLE, outSpeed = 0, outHeadingDeg = 0;
  if (originSet) {
    outLat = lat0 + (inavX[1] / EARTH_RADIUS_M) * RAD_TO_DEG;
    outLon = lon0 + (inavX[0] / (EARTH_RADIUS_M * cosLat0)) * RAD_TO_DEG;
    outSpeed = inavX[2];
    outHeadingDeg = inavX[3] * RAD_TO_DEG;
    if (outHeadingDeg < 0) outHeadingDeg += 360.0f;
    // INAV heading remains the GPS/EKF navigation heading. V10 deliberately
    // uses the independent relative IMU yaw below so rotation is visible even
    // when GPS has no fix.
    if (gps.altitude.isValid()) inavLastAlt = (float)gps.altitude.meters();
    outAlt = inavLastAlt;   // held from last valid GPS fix — no IMU altitude dead-reckoning
  }

  angle[2] = imuYawDeg;

  if (dataMutexTake(10) == pdTRUE) {
    // sd.pps_locked is NOT written here anymore — it's updated unconditionally
    // in sensorTask's main mutex block, since PPS is independent of BMI160
    // and this function returns early whenever BMI160 isn't ready.
    sd.inav_state      = state;
    sd.inav_lat        = outLat;
    sd.inav_lng        = outLon;
    sd.inav_alt        = outAlt;
    sd.inav_speed_mps  = outSpeed;
    sd.inav_heading_deg= outHeadingDeg;
    // v2.4: BMI160 also feeds V8-V11 (Roll/Pitch/Yaw/temp), replacing JY-901.
    if (bmiOk && !imuStuck) {
      sd.angle[0] = angle[0]; sd.angle[1] = angle[1]; sd.angle[2] = angle[2];
      sd.imu_temp = imuT;
      sd.imu_ok = true;
      sd.status_flags |= STATUS_IMU_OK;
      sd.status_flags &= ~STATUS_IMU_STUCK;
    }
    if (imuStuck) {
      sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
      sd.imu_ok = false;
      sd.status_flags &= ~STATUS_IMU_OK;
      sd.status_flags |= STATUS_IMU_STUCK;
    }
    xSemaphoreGive(dataMutex);
  }
  if (imuStuck) engMsgf("IMU STUCK: BMI160 no good read for %lus", (millis()-lastBmiOkMillis)/1000);
  perfRecordMax(perfMaxI2cPathMs,
                (uint32_t)((micros() - perfInavStartUs) / 1000UL));
}

// ============================================================================
// SENSOR TASK — Core 1 (app_cpu)
// All blocking sensor I/O lives here, away from Blynk/WiFi/lwIP on Core 0.
// Uses vTaskDelay for all waits so FreeRTOS tick ISR is never starved.
// NOTE: ADC is NOT reconfigured here — it was already configured globally
// in setup() BEFORE I2S init. Reconfiguring ADC after I2S install causes
// the "ADC: CONFLICT driver_ng" error.
// v2.2: this ordering constraint applied to the legacy I2S driver. Since
// INMP441 now uses <driver/i2s_std.h> (see initINMP441()), the constraint
// no longer applies — analogRead() calls in this task cannot collide with
// I2S regardless of when initINMP441() was called in setup(). The original
// ordering is kept anyway as good practice, not because it's still required.
// ============================================================================

// Initialize every sensor-facing driver from Core 1. Driver installation can
// bind interrupts/helper tasks to the calling core, so moving only the polling
// loop would not create the requested isolation.
static volatile bool sensorHardwareReady = false;

static void sensorOutputTask(void* pvParam)
{
  while (!setupDone || !sensorHardwareReady) vTaskDelay(pdMS_TO_TICKS(10));
  for (;;) {
    BuzzerCommand command;
    if (buzzerCommandQueue != NULL &&
        xQueueReceive(buzzerCommandQueue, &command, 0) == pdTRUE) {
      if (command.type == BUZZER_CMD_PATTERN3)
        buzzerPattern3(command.frequency, command.durationMs, command.pauseMs);
      else
        buzzerTone(command.frequency, command.durationMs);
    }
    buzzerService();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void sensorHardwareInit()
{
  sensorLogPrintf("[TASK] sensor hardware init on Core %d\n", xPortGetCoreID());

  pinMode(ledPin11,OUTPUT); digitalWrite(ledPin11,LOW);
  pinMode(ledPin12,OUTPUT); digitalWrite(ledPin12,LOW);
  pinMode(ledPin13,OUTPUT); digitalWrite(ledPin13,LOW);
  pinMode(PPS_LED_PIN, OUTPUT); setPpsLed(false);
  if (DUSTsensorThere) {
    pinMode(dustLED, OUTPUT); digitalWrite(dustLED, HIGH);
  }
  pinMode(buzzPin, OUTPUT);
  if (COsensorThere) {
    pinMode(CO_ADC_PIN, INPUT);
    pinMode(CO_A1_PIN, INPUT);
  }
  pinMode(UV_OUT_PIN, INPUT);
  pinMode(UV_REF_PIN, INPUT);
  if (DUSTsensorThere) pinMode(DUST_ADC_PIN, INPUT);

  buzzerLEDCReady = ledcAttachChannelSafe(buzzPin, 5000, 8, LEDC_CHANNEL_BUZZER);
  if (buzzerLEDCReady) {
    ledcWriteSafe(buzzPin, 0);
    sensorLogPrintf("[INIT] Buzzer LEDC attached on channel 8 (group 1)\n");
  } else {
    sensorLogPrintf("[WARN] Buzzer LEDC attach failed - buzzer disabled\n");
  }

  analogReadResolution(12);
  if (COsensorThere) {
    analogSetPinAttenuation(CO_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(CO_A1_PIN, ADC_11db);
  }
  analogSetPinAttenuation(UV_OUT_PIN, ADC_11db);
  analogSetPinAttenuation(UV_REF_PIN, ADC_11db);
  if (DUSTsensorThere) analogSetPinAttenuation(DUST_ADC_PIN, ADC_11db);
  sensorLogPrintf("[INIT] ADC configured on sensor core (12-bit, per-pin 11dB)\n");
  sensorLogPrintf("[INIT] Optional analog sensors: MQ-7=%s GP2Y1010=%s\n",
                  COsensorThere ? "ENABLED" : "NOT INSTALLED",
                  DUSTsensorThere ? "ENABLED" : "NOT INSTALLED");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  sensorLogPrintf("[I2C DIAG] Startup scan: 0x38(AHT2x), 0x53(ENS160), 0x68(BMI160), 0x69(BMI160)\n");
  for (uint8_t diagAddr = 0x08; diagAddr <= 0x77; ++diagAddr) {
    if (diagAddr == 0x38 || diagAddr == 0x53 || diagAddr == 0x68 || diagAddr == 0x69) {
      uint32_t diagStartUs = micros();
      bool present = i2cDevicePresent(diagAddr);
      DBG_PRINTF("[I2C DIAG] scan 0x%02X -> %s (%lums err=%u SDA=%d SCL=%d)\n",
                 diagAddr, present ? "ACK" : "NO ACK",
                 (unsigned long)((micros() - diagStartUs) / 1000UL),
                 i2cLastWireError, i2cLastSdaLevel, i2cLastSclLevel);
    }
  }

  if (PPSsensorThere) {
    pinMode(PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsISR, RISING);
    sensorLogPrintf("[INIT] GPS PPS interrupt attached on sensor Core 1 (GPIO5)\n");
  }

  if (COsensorThere) {
    ledcAttachChannelSafe(CO_PWM_PIN, LEDC_FREQ_CO, LEDC_RES_CO, LEDC_CHANNEL_CO);
    ledcWriteSafe(CO_PWM_PIN, 0);
  }

  initGPS();
  if (DISPLAYON) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
      sensorLogPrintf("[WARN] OLED init failed\n");
    else { display.setTextColor(WHITE); display.clearDisplay(); display.display(); }
  }
  randomSeed(analogRead(0));
  esp_task_wdt_reset();

  if (INMPsensorThere) initINMP441();
  esp_task_wdt_reset();

  if (COsensorThere) {
    sens_val = sensor_reading_clean_air;
    if (!coSensorPresent()) {
      sensorLogPrintf("[WARN] CO sensor not detected (GPIO36 floating)\n");
      engMsg("CO: sensor absent - check A0 wiring and divider");
      if (dataMutexTake(20)==pdTRUE) {
        sd.co_no_sensor=true; sd.status_flags|=STATUS_CO_NO_SNS;
        sd.co_ppm=SENSOR_UNAVAILABLE; xSemaphoreGive(dataMutex);
      }
    } else {
      engMsg("CO: sensor detected - starting closed-loop heater control");
    }
    startMeasurementPhase();
  } else {
    sensorLogPrintf("[INIT] CO: NOT INSTALLED; PWM and ADC sampling disabled\n");
    if (dataMutexTake(20) == pdTRUE) {
      sd.co_no_sensor = true;
      sd.co_fault = false;
      sd.status_flags = (sd.status_flags | STATUS_CO_NO_SNS) & ~STATUS_CO_FAULT;
      sd.co_ppm = sd.co_raw = sd.co_heater_v = SENSOR_UNAVAILABLE;
      xSemaphoreGive(dataMutex);
    }
  }
  sensorHardwareReady = true;
  sensorLogPrintf("[TASK] sensor hardware init complete on Core 1\n");
}

static inline void sensorStageRecord(const char* name, uint32_t elapsedMs, uint32_t* maxPtr)
{
  if (elapsedMs > *maxPtr) *maxPtr = elapsedMs;
  if (elapsedMs >= 100UL) {
    ++sensorStageWarnCount;
    DBG_PRINTF("[SENSOR STAGE] %s=%lums (aggregate cycle may starve Core-0 service)\n",
               name, (unsigned long)elapsedMs);
  }
}

void sensorTask(void* pvParam)
{
  while (!setupDone) vTaskDelay(pdMS_TO_TICKS(10));

  sensorLogPrintf("[TASK] sensorTask running on Core %d, priority %u\n",
                xPortGetCoreID(), (unsigned)uxTaskPriorityGet(NULL));

  // Subscribe the sensor task to the ESP task watchdog. The task feeds it only
  // after a complete sensor cycle, so a genuinely stuck sensor/I2C operation
  // can no longer leave the station silently wedged for an unlimited time.
  // The subscription/feed API itself is unchanged in ESP-IDF 5.x.
  esp_err_t wdtResult = esp_task_wdt_add(NULL);
  if (wdtResult != ESP_OK && wdtResult != ESP_ERR_INVALID_STATE) {
    sensorLogPrintf("[WATCHDOG] sensorTask registration failed: %d\n", (int)wdtResult);
  }

  sensorHardwareInit();

  // ADC config already done in setup() — do NOT call analogReadResolution()
  // or analogSetAttenuation() here. That would trigger the driver conflict.

  while (true)
  {
    const uint32_t requestedRecovery = sensorRecoveryRequestSequence;
    if (requestedRecovery != sensorRecoveryAckSequence) {
      sensorLogPrintf("[SENSOR VIABILITY] recovery request #%lu received on Core 1\n",
                      (unsigned long)requestedRecovery);
      esp_task_wdt_reset();

      // Cooperate only at this cycle boundary, where sensorTask owns no mutex
      // and no I2C transaction is active. Rebuild the shared sensor bus and
      // invalidate library readiness so both devices are initialized afresh.
      lastI2CRecovery = millis() - I2C_RECOVERY_MIN_INTERVAL_MS;
      i2cBusRecover();
      ensReady = false;
      bmi160Ready = false;
      ens_fail_count = 0;
      ens_i2c_fail_streak = 0;
      const bool ensRecovered = !ENSsensorThere || initENS();
      const bool bmiRecovered = !BMI160sensorThere || initBMI160();

      ++sensorRecoveryCount;
      sensorRecoveryAckSequence = requestedRecovery;
      ++sensorTaskHeartbeat;
      esp_task_wdt_reset();
      sensorLogPrintf("[SENSOR VIABILITY] recovery #%lu acknowledged ENS=%d BMI=%d I2Crecoveries=%lu\n",
                      (unsigned long)requestedRecovery,
                      ensRecovered ? 1 : 0,
                      bmiRecovered ? 1 : 0,
                      (unsigned long)i2cBusRecoveryCount);
    }

    sensorTaskState = SENSOR_STATE_RUN;
    uint32_t perfSensorCycleStartUs = micros();

    unsigned long loopStart = millis();

    // ── GPS PPS LED — keep the ESP-32 board D2 LED on for exactly 250ms ───────
    // Historical implementation note retained from v2.6:
    // The PPS ISR turns the LED on immediately; this task turns it off after
    // the requested 0.25 second pulse. PPS timing itself remains ISR-driven.
    // In v2.7 the ISR no longer drives the LED directly; it only records the
    // event, and this task performs both the ON and OFF operations.
    // The PPS ISR only records the event. This task performs the actual LED
    // operation, so the LED now also proves that the normal sensor application
    // is running rather than merely proving that the interrupt still fires.
    // A counter is used instead of a boolean so closely spaced/queued events
    // cannot be silently lost; normal GPS PPS is one event per second.
    // ── GPS PPS LED ───────────────────────────────────────────────────────────
    // v3.6: LED timing is no longer serviced by sensorTask. A dedicated PPS LED
    // task is woken by the GPIO5 PPS interrupt and generates the exact 250ms
    // pulse. This prevents slow sensor/I2C processing from stretching the pulse.
    // The LED is strictly event-driven: it is ON only for the fixed 250ms PPS
    // indication period, and is never held ON merely because PPS remains locked.
    // PPS lock state is deliberately NOT used here: every physical PPS event
    // produces one LED pulse, while loss of PPS naturally leaves the LED OFF.

    // ── UV (ML8511 3.3V ratiometric, direct — no divider) ───────────────────
    int uvRaw = averageAnalogRead(UV_OUT_PIN);
    int uvRef = averageAnalogRead(UV_REF_PIN);
    float uvi = SENSOR_UNAVAILABLE;
    if (uvRef > 50) {
      float outV = (3.3f / (float)uvRef) * (float)uvRaw;
      uvi = constrain(12.49f*(outV+0.03f)-12.49f+0.3f, 0.0f, 20.0f);
    }

    // ── Dust (GP2Y1010 5V, 10k+10k divider) ─────────────────────────────────
    // GPIO18 drives IR LED at 3.3V — safe direct connect to sensor ILED pin.
    // Never sample an unconnected ADC: a floating ESP32 input can look like a
    // valid changing signal and cannot establish that hardware is present.
    int voMeas = -1;
    float dustDens = SENSOR_UNAVAILABLE;
    if (DUSTsensorThere) {
      digitalWrite(dustLED, LOW);
      delayMicroseconds(280);
      voMeas = analogRead(DUST_ADC_PIN);
      delayMicroseconds(40);
      digitalWrite(dustLED, HIGH);
      dustDens = max(0.0f, 0.17f * adcToSensorVoltage(voMeas) - 0.1f);
    }

    // ── Additive sensor-health diagnostics ────────────────────────────────────
    // Persistent rail/no-data conditions are reported, but the corresponding
    // sensor value remains SENSOR_UNAVAILABLE and all other sensors continue.
    // A single bad sample is deliberately ignored to avoid false alarms.
    if (!inmpLastReadHadData) {
      if (inmpFaultCount < SENSOR_HEALTH_FAIL_LIMIT) inmpFaultCount++;
    } else {
      if (inmpFaultReported) engMsg("INMP441: data recovered");
      inmpFaultCount = 0;
      inmpFaultReported = false;
    }
    if (inmpFaultCount >= SENSOR_HEALTH_FAIL_LIMIT) {
      inmpFaultReported = true;
      sensorHealthMsg("INMP441", "no I2S samples", lastInmpHealthMsg);
    }

    bool uvInvalid = (uvRef <= 50);
    if (uvInvalid) {
      if (uvFaultCount < SENSOR_HEALTH_FAIL_LIMIT) uvFaultCount++;
    } else {
      if (uvFaultReported) engMsg("UV: sensor data recovered");
      uvFaultCount = 0;
      uvFaultReported = false;
    }
    if (uvFaultCount >= SENSOR_HEALTH_FAIL_LIMIT) {
      uvFaultReported = true;
      sensorHealthMsg("UV", "invalid reference signal", lastUvHealthMsg);
    }

    if (DUSTsensorThere) {
      bool dustRailFault = (voMeas <= 2 || voMeas >= 4093);
      if (dustRailFault) {
        if (dustFaultCount < SENSOR_HEALTH_FAIL_LIMIT) dustFaultCount++;
      } else {
        if (dustFaultReported) engMsg("Dust: ADC data recovered");
        dustFaultCount = 0;
        dustFaultReported = false;
      }
      if (dustFaultCount >= SENSOR_HEALTH_FAIL_LIMIT) {
        dustFaultReported = true;
        sensorHealthMsg("Dust", "ADC input at rail", lastDustHealthMsg);
        dustDens = SENSOR_UNAVAILABLE;
      }
    }

    // ── ENS160 + AHT2x (I2C, ScioSense library) ─────────────────────────────
    float tvoc=SENSOR_UNAVAILABLE, eco2=SENSOR_UNAVAILABLE, aqi=SENSOR_UNAVAILABLE;
    float t=temp_prev, h=hum_prev;
    bool ensOK = false;
    bool ensFaulted = (ens_fail_count >= ENS_MAX_FAILS);
    uint32_t sensorEnsStartMs = millis();
#if ESP32_RX_ISOLATION_TEST
    // Network-isolation run: deliberately bypass ENS160/AHT2x I2C workload.
    // The normal full implementation remains intact below for the default mode.
#else
    if (!ensFaulted || (millis()-lastENSattempt > ENS_RETRY_MS)) {
      lastENSattempt = millis();
      if (!ensReady) {
        // Attempt (re)init — retried from sensorTask in case board was absent at boot
        ensReady = initENS();
      }
      if (ensReady) {
        ensOK = readENS(tvoc, eco2, aqi, t, h);
        if (ensOK) {
          temp_prev=t; hum_prev=h;
          ens_fail_count=0;
          ens_i2c_fail_streak=0;
        } else {
          ens_fail_count++;
          t=temp_prev; h=hum_prev;
          if (ens_i2c_fail_streak < 255) ens_i2c_fail_streak++;
          // A failed preflight/read means the physical device or I2C bus may
          // have disappeared. Do not keep a stale "ready" library state.
          if (ens_i2c_fail_streak >= ENS_I2C_REINIT_FAILS) {
            ensReady = false;
            DBG_PRINTF("[I2C RECOVERY] ENS160/AHT2x marked not-ready after %u consecutive failed cycles — reinitializing\n",
                       (unsigned)ens_i2c_fail_streak);
            i2cBusRecover();
          }
        }
      } else {
        ens_fail_count++;
      }
      if (ens_fail_count == ENS_MAX_FAILS)
        engMsgf("ENS FAULT: %d failures — retrying every %ds", ENS_MAX_FAILS, ENS_RETRY_MS/1000);
    }
    if (ensFaulted && !ensOK) { t=SENSOR_UNAVAILABLE; h=SENSOR_UNAVAILABLE; }
#endif
    sensorStageRecord("ENS160/AHT2x", millis() - sensorEnsStartMs, &sensorStageMaxEnsMs);

    // ── INMP441 I2S mic ───────────────────────────────────────────────────────
    uint32_t sensorInmpStartMs = millis();
#if ESP32_RX_ISOLATION_TEST
    float soundDb = SENSOR_UNAVAILABLE;
#else
    float soundDb = INMPsensorThere ? readINMP441_dB() : SENSOR_UNAVAILABLE;
#endif
    sensorStageRecord("INMP441", millis() - sensorInmpStartMs, &sensorStageMaxInmpMs);

    // ── CO state machine ─────────────────────────────────────────────────────
    uint32_t sensorCoStartMs = millis();
#if !ESP32_RX_ISOLATION_TEST
    if (COsensorThere) tickCO();
#endif
    sensorStageRecord("CO", millis() - sensorCoStartMs, &sensorStageMaxCoMs);

    // ── GPS (ATGM336H NMEA, 9600 baud) ───────────────────────────────────────
    uint32_t sensorGpsStartMs = millis();
    feedGPS();
    // v3.5: Snapshot this BEFORE gps.location.lat()/lng() are read below.
    // TinyGPS++ clears location.isUpdated() when lat()/lng() are accessed,
    // and INAV needs to know that this cycle contains a fresh GPS position.
    gpsLocationUpdatedThisCycle = gps.location.isUpdated();
    SatGPS = (float)gps.satellites.value();
    HDOP   = (SatGPS >= 1) ? gps.hdop.value()/100.0f : 99.9f;
    bool gpsFix = (gps.location.isValid() &&
                   gps.location.age() < GPS_TIMEOUT_MS && SatGPS >= 4);
    double lat=0, lng=0;
    if (gpsFix) {
      lat=gps.location.lat(); lng=gps.location.lng();
      lastGPSdata=millis(); prevZeroVal=false;
    } else {
      if (!prevZeroVal) {
        lat=gps.location.lat()+0.0001; lng=gps.location.lng()+0.0001;
        prevZeroVal=true;
        engMsgf("GPS: fix lost (sats=%.0f age=%lums)", SatGPS, gps.location.age());
      } else { lat=0; lng=0; }
      if (millis()-lastGPSbeep > BUZZER_GPS_INTERVAL) {
        lastGPSbeep=millis(); buzzerTone(12000, 90);
      }
    }
    if (lastGPSdata>0 && millis()-lastGPSdata>GPS_TIMEOUT_MS)
      engMsgf("GPS FAULT: no data >%ds", GPS_TIMEOUT_MS/1000);

    // v2.5: computed here (Core 0, sensorTask) where `gps` is safely owned —
    // never called from loop()/Core 1. Cached to sd.gps_unix_time below.
    long gpsUnixT = gpsUnixTime();
    sensorStageRecord("GPS", millis() - sensorGpsStartMs, &sensorStageMaxGpsMs);

    // ── Inertial navigation — GPS + BMI160 EKF (v2.4) ────────────────────────
    // BMI160 is now the ONLY IMU. This one call handles everything: EKF
    // dead-reckoning, PPS/lat/lng/speed/heading, AND (via the complementary
    // filter inside inertialNav()) Roll/Pitch/Yaw/temp for sd.angle[]/
    // sd.imu_temp — the job JY-901's readIMU() used to do, now gone.
    uint32_t sensorInavStartMs = millis();
#if ESP32_RX_ISOLATION_TEST
    if (dataMutexTake(5) == pdTRUE) {
      sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
      xSemaphoreGive(dataMutex);
    }
#else
    if (BMI160sensorThere) inertialNav();
    else if (dataMutexTake(5) == pdTRUE) {
      sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
      xSemaphoreGive(dataMutex);
    }
#endif
    sensorStageRecord("INAV/BMI160", millis() - sensorInavStartMs, &sensorStageMaxInavMs);

    // ── LED control ──────────────────────────────────────────────────────────
    float cppm = COsensorThere ? last_CO_ppm : SENSOR_UNAVAILABLE;
    digitalWrite(ledPin11, (COsensorThere && cppm>=0 && cppm<=10) ? HIGH : LOW);
    digitalWrite(ledPin12, (COsensorThere && cppm>10 && cppm<=20) ? HIGH : LOW);
    digitalWrite(ledPin13, (COsensorThere && cppm>20)             ? HIGH : LOW);

    // ── OLED ─────────────────────────────────────────────────────────────────
    uint32_t sensorOledStartMs = millis();
#if !ESP32_RX_ISOLATION_TEST
    if (DISPLAYON) {
      display.clearDisplay(); display.setCursor(0,0); display.setTextSize(1);
      display.printf("CO:%.1f UV:%.1f\n", cppm, uvi);
      display.printf("T:%.1fC H:%.0f%%\n", t, h);
      display.printf("TVOC:%.0fppb CO2:%.0f\n", tvoc, eco2);
      display.printf("%.6f\n%.6f\n", lat, lng);
      if (BMI160sensorThere && sd.imu_ok)
        display.printf("R/P/Y:%.0f/%.0f/%.0f\n", angle[0],angle[1],angle[2]);
      display.display();
    }
#endif
    sensorStageRecord("OLED", millis() - sensorOledStartMs, &sensorStageMaxOledMs);

    // ── Single mutex-protected struct write ──────────────────────────────────
    // Mutex held only here — never during I/O operations above.
    uint32_t flags = 0;
    if (ensOK)                                               flags |= STATUS_ENS_OK;
    if (ens_fail_count>=ENS_MAX_FAILS)                       flags |= STATUS_ENS_FAULT;
    if (gpsFix)                                              flags |= STATUS_GPS_OK;
    if (lastGPSdata>0&&millis()-lastGPSdata>GPS_TIMEOUT_MS) flags |= STATUS_GPS_FAULT;

    SensorData completedSnapshot;
    bool completedSnapshotReady = false;
    if (dataMutexTake(20) == pdTRUE) {
      sd.uvi=uvi; sd.dust_mg=dustDens;
      sd.tvoc_ppb=tvoc; sd.eco2_ppm=eco2; sd.aqi=aqi;
      sd.sound_db=soundDb;
      sd.temp=t; sd.hum=h; sd.ens_fault=(ens_fail_count>=ENS_MAX_FAILS);
      sd.lat=lat; sd.lng=lng; sd.hdop=HDOP; sd.sats=SatGPS; sd.gps_fix=gpsFix;
      sd.gps_unix_time = gpsUnixT;
      sd.rand_num=random(1,10);
      // BUGFIX: sd.pps_locked was previously only ever written inside
      // inertialNav(), which returns early (before reaching that line)
      // whenever BMI160 isn't ready. PPS hardware has nothing to do with
      // BMI160, so it must be updated here unconditionally instead.
      sd.pps_locked = ppsIsLocked();
      // Preserve WiFi/CO/IMU flags already set by their own functions; merge sensor flags
      sd.status_flags = (sd.status_flags &
        (STATUS_WIFI_OK|STATUS_IMU_OK|STATUS_IMU_STUCK|
         STATUS_CO_FAULT|STATUS_CO_NO_SNS)) | flags;
      completedSnapshot = sd;
      completedSnapshotReady = true;
      xSemaphoreGive(dataMutex);
    }
    if (completedSnapshotReady && sensorSnapshotQueue != NULL) {
      // Queue length is one: replace the old value atomically and never wait.
      xQueueOverwrite(sensorSnapshotQueue, &completedSnapshot);
      // Seal the authoritative V61/V62 strings here on Core 1. This call only
      // formats local data and performs a zero-wait overwrite queue copy.
      core1PublishTelemetryPair(completedSnapshot);
    }

    if (DEBUGON)
      DBG_PRINTF("[SENS] CO=%.1f ph=%d UV=%.2f Snd=%.1fdB "
                    "Dst=%.3f T=%.1f H=%.0f TVOC=%.0f eCO2=%.0f AQI=%.0f "
                    "GPS=%d sat=%.0f\n",
                    last_CO_ppm,co_phase,uvi,soundDb,dustDens,
                    t,h,tvoc,eco2,aqi,(int)gpsFix,SatGPS);

    // ── 10-min optional auto-reset ────────────────────────────────────────────
    countReset++;
    if (ten_mins_autoreset && countReset >= 600) {
      engMsg("AUTO-RESET: 10-min watchdog");
      vTaskDelay(pdMS_TO_TICKS(300));
      buzzerTone(1000, 200);
      countReset=0; diagnosticResetRequest("TEN_MIN_AUTORESET");
    }

    // Sensor-task health heartbeat. Updated only after the complete cycle
    // finishes, so a blocked sensor operation cannot falsely report health.
    sensorTaskHeartbeat++;

    // Feed the ESP task watchdog after a successful full sensor cycle. If the
    // task becomes stuck in an I2C/sensor operation beyond the watchdog window,
    // the hardware/software watchdog infrastructure can recover the ESP32.
    esp_task_wdt_reset();

    unsigned long elapsed = millis()-loopStart;
    perfRecordMax(perfMaxSensorCycleMs,
                  (uint32_t)((micros() - perfSensorCycleStartUs) / 1000UL));

    SensorDiagnosticsSnapshot diag = {};
    diag.capturedAtMs = millis();
    diag.heartbeat = sensorTaskHeartbeat;
    diag.ppsCount = ppsPulseCount;
    diag.ppsAgeMs = perfPpsAgeMs();
    diag.maxSensorCycleMs = perfMaxSensorCycleMs;
    diag.maxGpsFeedMs = perfMaxGpsFeedMs;
    diag.maxI2cPathMs = perfMaxI2cPathMs;
    diag.maxPpsLedResponseMs = perfMaxPpsLedResponseMs;
    diag.lastI2cDevice = perfLastI2cDevice;
    diag.lastI2cOperation = perfLastI2cOperation;
    diag.i2cErrorCount = perfI2cErrorCount;
    diag.maxAht21Ms = perfMaxAht21Ms;
    diag.maxEns160Ms = perfMaxEns160Ms;
    diag.maxBmi160Ms = perfMaxBmi160Ms;
    diag.stageMaxEnsMs = sensorStageMaxEnsMs;
    diag.stageMaxInmpMs = sensorStageMaxInmpMs;
    diag.stageMaxCoMs = sensorStageMaxCoMs;
    diag.stageMaxGpsMs = sensorStageMaxGpsMs;
    diag.stageMaxInavMs = sensorStageMaxInavMs;
    diag.stageMaxOledMs = sensorStageMaxOledMs;
    diag.stageWarnCount = sensorStageWarnCount;
    diag.dataMutexCalls = dataMutexTakeCalls;
    diag.dataMutexTimeouts = dataMutexTakeTimeouts;
    diag.dataMutexMaxWaitUs = dataMutexMaxWaitUs;
    diag.ledcLockCalls = ledcLockTakeCalls;
    diag.ledcLockTimeouts = ledcLockTimeouts;
    diag.ledcLockMaxWaitUs = ledcLockMaxWaitUs;
    diag.i2cProbeFailCount = i2cProbeFailCount;
    diag.i2cRecoveryCount = i2cBusRecoveryCount;
    diag.i2cSdaLevel = i2cLastSdaLevel;
    diag.i2cSclLevel = i2cLastSclLevel;
    diag.sensorState = sensorTaskState;
    diag.coPhase = co_phase;
    diag.coDuty = co_duty;
    diag.coHeaterV = lastCoHeaterV;
    if (sensorDiagnosticsQueue != NULL) xQueueOverwrite(sensorDiagnosticsQueue, &diag);

    vTaskDelay(pdMS_TO_TICKS(elapsed < SENSOR_TASK_PERIOD_MS
                              ? SENSOR_TASK_PERIOD_MS - elapsed : 1));
  }
}

// ============================================================================
// BLYNK SENDS — called from BlynkTimer in loop() (communications Core 0)
// ============================================================================

// v3.9.93: one legacy dashboard channel per second, round-robin.  This replaces the
// old FAST/SLOW bursts as the normal legacy-dashboard publisher.  V61/V62 remain
// the authoritative 1 Hz coherent database/GIS record.
void blynkSendDashboardRoundRobin()
{
  if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) return;
  if (blynkTxCongested()) return;

  int32_t rssi = WiFi.RSSI();
  uint8_t qual = rssiToQuality(rssi);
  SensorData snap;
  if (!blynkGetSensorSnapshot(snap)) return;
  snap.rssi = rssi;
  snap.wifi_qual = qual;

  switch (blynkDashboardRoundRobinIndex) {
    case 0:  safeWrite (V1,  snap.sound_db); break;
    case 1:  safeWrite (V2,  snap.co_ppm); break;
    case 2:  safeWrite (V3,  snap.uvi); break;
    case 3:  safeWrite (V5,  snap.tvoc_ppb); break;
    case 4:  safeWriteI(V6,  snap.co_phase); break;
    case 5:  safeWrite (V7,  snap.co_raw); break;
    case 6:  safeWrite (V8,  snap.angle[0]); break;
    case 7:  safeWrite (V9,  snap.angle[1]); break;
    // V10 = relative BMI160 Z-axis yaw; unlike the old GPS/EKF mirror it moves
    // immediately when the unit rotates, even with GPS unavailable.
    case 8:  safeWrite (V10, snap.angle[2]); break;
    case 9:  safeWrite (V11, snap.imu_temp); break;
    case 10: safeWrite (V12, snap.lat); break;
    case 11: safeWrite (V13, snap.lng); break;
    case 12: safeWriteI(V14, (int)snap.sats); break;
    case 13: safeWrite (V15, snap.hum); break;
    case 14: safeWrite (V16, snap.temp); break;
    case 15: safeWrite (V17, snap.hdop); break;
    case 16: safeWrite (V18, snap.dust_mg); break;
    case 17: safeWriteS(V19, snap.eng_msg); break;
    case 18: safeWriteI(V20, (int)snap.status_flags); break;
    case 19: safeWriteI(V21, snap.rssi); break;
    case 20: safeWriteI(V22, (int)snap.wifi_qual); break;
    case 21: safeWrite (V23, snap.eco2_ppm); break;
    case 22: safeWrite (V24, snap.aqi); break;
    case 23: safeWrite (V25, snap.co_heater_v); break;
    case 24: safeWriteI(V26, snap.pps_locked ? 1 : 0); break;
    case 25: safeWriteI(V27, snap.inav_state); break;
    case 26: safeWrite (V28, snap.inav_lat); break;
    case 27: safeWrite (V29, snap.inav_lng); break;
    case 28: safeWrite (V30, snap.inav_alt); break;
    case 29: safeWrite (V31, snap.inav_speed_mps); break;
    case 30: safeWrite (V32, snap.inav_heading_deg); break;
  }
  blynkDashboardRoundRobinIndex = (uint8_t)((blynkDashboardRoundRobinIndex + 1U) % 31U);
}

// v3.9.93: diagnostics are also spread out.  One diagnostic channel every
// two seconds means V34-V65 are refreshed over about one minute instead of a
// burst of ~30 writes that can itself create a send backlog.
void blynkSendDiagnosticsRoundRobin()
{
  if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) return;
  if (blynkTxCongested(BLYNK_TX_HARD_LIMIT)) return;

  switch (blynkDiagRoundRobinIndex) {
    case 0:  safeWriteS(V34, bootResetReasonText); break;
    case 1:  safeWriteI(V35, (int)core0SensorDiag.ppsCount); break;
    case 2:  safeWriteI(V36, (int)core0SensorDiag.ppsAgeMs); break;
    case 3:  safeWriteI(V37, (int)perfMaxLoopGapMs); break;
    case 4:  safeWriteI(V38, (int)core0SensorDiag.maxSensorCycleMs); break;
    case 5:  safeWriteI(V39, (int)perfMaxBlynkRunMs); break;
    case 6:  safeWriteI(V40, (int)core0SensorDiag.maxGpsFeedMs); break;
    case 7:  safeWriteI(V41, (int)core0SensorDiag.maxI2cPathMs); break;
    case 8:  safeWriteI(V42, (int)perfMaxFastSendMs); break;
    case 9:  safeWriteI(V43, (int)perfMaxSlowSendMs); break;
    case 10: safeWriteI(V44, (int)core0SensorDiag.maxPpsLedResponseMs); break;
    case 11: safeWriteI(V45, (int)core0SensorDiag.lastI2cDevice); break;
    case 12: safeWriteI(V46, (int)core0SensorDiag.lastI2cOperation); break;
    case 13: safeWriteI(V47, (int)core0SensorDiag.i2cErrorCount); break;
    case 14: safeWriteI(V48, (int)core0SensorDiag.maxAht21Ms); break;
    case 15: safeWriteI(V49, (int)core0SensorDiag.maxEns160Ms); break;
    case 16: safeWriteI(V50, (int)core0SensorDiag.maxBmi160Ms); break;
    case 17: { char rtcText[512]; rtcFormatPreviousRun(rtcText, sizeof(rtcText)); safeWriteS(V51, rtcText); break; }
    case 18: safeWriteS(V52, core1StageText(core1StageId)); break;
    case 19: safeWriteI(V53, (int)core1MaxStageMs); break;
    case 20: safeWriteI(V54, (int)core1StallCount); break;
    case 21: safeWriteI(V56, (int)blynkTxLastVPin); break;
    case 22: safeWriteI(V57, (int)blynkTxLastCallMs); break;
    case 23: safeWriteI(V58, (int)blynkTxMaxCallMs); break;
    case 24: safeWriteI(V59, (int)blynkTxCallCount); break;
    case 25: safeWriteI(V60, (int)blynkTxLastValueLen); break;
    case 26: safeWriteI(V63, (int)blynkTxPendingCount()); break;
    case 27: safeWriteI(V64, (int)blynkTxLastVPin); break;
    case 28: safeWriteI(V65, (int)blynkTxMaxCallMs); break;
  }
  blynkDiagRoundRobinIndex = (uint8_t)((blynkDiagRoundRobinIndex + 1U) % 29U);
}

void blynkSendFast()
{
  core1StageBegin(CORE1_STAGE_BLYNK_TIMER);
  uint32_t perfFastStartUs = micros();

  // Critical / navigation dashboard data — sent every BLYNK_SEND_FAST_MS (20s)
  if (WiFi.status() != WL_CONNECTED) { core1StageEnd(); return; }
  int32_t rssi = WiFi.RSSI();
  uint8_t qual = rssiToQuality(rssi);
  SensorData snap;
  if (!blynkGetSensorSnapshot(snap)) { core1StageEnd(); return; }
  snap.rssi=rssi; snap.wifi_qual=qual;

  // v3.9.91: V61 is authoritative for database/GIS ingestion. These V-pins
  // remain available for the dashboard, but are sent at a lower cadence and
  // are skipped when the TX queue is already congested. This prevents the
  // dashboard from feeding an outage backlog while V4/V61 retain priority.
  if (blynkTxCongested()) {
    blynkTxCongestionSkips++;
    core1StageEnd();
    return;
  }

  // v3.9.92: explicitly refresh EVERY legacy dashboard V-pin that belongs
  // to the FAST/navigation set. v3.9.91 accidentally omitted V15/V16/V17,
  // V19, V26 and V30-V32, so those widgets could remain stale forever.
  safeWrite (V2,  snap.co_ppm);
  safeWriteI(V6,  snap.co_phase);
  safeWrite (V7,  snap.co_raw);
  safeWrite (V12, snap.lat);
  safeWrite (V13, snap.lng);
  safeWriteI(V14, (int)snap.sats);
  safeWrite (V15, snap.hum);
  safeWrite (V16, snap.temp);
  safeWrite (V17, snap.hdop);
  safeWriteS(V19, snap.eng_msg);
  safeWriteI(V20, (int)snap.status_flags);
  safeWriteI(V21, snap.rssi);
  safeWriteI(V22, (int)snap.wifi_qual);
  safeWriteI(V26, snap.pps_locked ? 1 : 0);
  safeWriteI(V27, snap.inav_state);
  safeWrite (V28, snap.inav_lat);
  safeWrite (V29, snap.inav_lng);
  safeWrite (V30, snap.inav_alt);
  safeWrite (V31, snap.inav_speed_mps);
  safeWrite (V32, snap.inav_heading_deg);
  perfRecordMax(perfMaxFastSendMs,
                (uint32_t)((micros() - perfFastStartUs) / 1000UL));
  core1StageEnd();
}

void blynkQueueCoherentSnapshot(const Core1TelemetryPair& pair)
{
  // Blynk 0.6.1 builds each virtualWrite command in a fixed-size
  // BLYNK_MAX_SENDBYTES buffer (commonly 128 bytes). The complete sensor set
  // is too large for one legacy Blynk command, so v3.9.91 uses TWO compact CSV
  // records with the SAME gpsUnix/deviceMs pair. The backend can join V61 and
  // V62 on that pair and reconstruct one coherent sensor/GPS record without
  // relying on network arrival time.
  //
  // V61 fields:
  // gpsUnix,deviceMs,lat,lng,sats,hdop,gpsFix,co,coRaw,heaterV,coPhase,
  // coFault,coNoSensor,tvoc,eCO2,aqi,uvi,sound,dust
  //
  // V62 fields:
  // gpsUnix,deviceMs,temp,hum,ensFault,roll,pitch,yaw,imuTemp,imuOk,pps,
  // inavState,inavLat,inavLng,inavAlt,inavSpeed,inavHeading,status,rssi,wifiQual
  // Do not overwrite an active pair. V61 and V62 must retain exactly the
  // same timestamp until both messages have been transmitted.
  if (blynkCoherentPairActive) return;

  // Queue both halves as one immutable pair. If the second insertion ever
  // fails, abandon the first half too; never expose a mismatched pair.
  const bool queuedA = blynkTxQueueValue(V61, pair.v61);
  const bool queuedB = blynkTxQueueValue(V62, pair.v62);
  if (queuedA && queuedB) {
    blynkCoherentPairActive = true;
    blynkCoherentPairSecondDue = false;
    blynkCoherentPairDeviceMs = pair.deviceMs;
    blynkCoherentPairGpsTs = pair.gpsUnix;
    blynkCoherentPairSequence = pair.sequence;
    blynkLastAcceptedTelemetrySequence = pair.sequence;
  } else {
    blynkTxDropCoherentPair();
  }
}

// v3.9.104 FORENSIC V61-ONLY TEST
// Sends exactly one application datastream: V61. V62/V4/dashboard/diagnostic/map
// traffic is intentionally disabled for this experiment.
void blynkSendV61Only()
{
  core1StageBegin(CORE1_STAGE_BLYNK_TIMER);
  const uint32_t startUs = micros();

  if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
    core1StageEnd();
    return;
  }

  // v3.9.112: do not keep regenerating application TX while the response
  // freshness guard is active. Blynk.run() remains fully serviced; this only
  // prevents a stale V61 item from sitting in the queue waiting for a dead
  // server-side response path.
  if (!blynkStockApplicationTxAllowed(millis())) {
    core1StageEnd();
    return;
  }

  // v3.9.110 CHALLENGE VARIABLE: fixed-size, human-readable CSV-like payload.
  // Only the six-digit record number changes: 000001 -> ... -> 000009 -> 000001.
  // Everything after the first field remains byte-for-byte identical.
  // This deliberately exercises a materially larger payload than the prior
  // prior one-byte 0..9 test while keeping content deterministic and easy to verify.
  char packet[64];
  const uint8_t testValue = v61TestValue;
  const uint32_t seq = ++v61TestSequence;
  const unsigned long queuedAt = millis();
  snprintf(packet, sizeof(packet),
           "%06u, test, 123.00000, -89.00000, 12.00, 99.88",
           (unsigned)testValue);
  v61TestValue = (uint8_t)((v61TestValue % 9U) + 1U);
  const bool queued = blynkTxQueueValue(V61, packet);
  Serial.printf("[V61 TEST] QUEUE seq=%lu record=%06u len=%u queued=%d at=%lums payload=\"%s\"\n",
                (unsigned long)seq, (unsigned)testValue, (unsigned)strlen(packet),
                queued ? 1 : 0, queuedAt, packet);

  if (!queued) {
    ++blynkTxCongestionSkips;
  }

  perfRecordMax(perfMaxSlowSendMs,
                (uint32_t)((micros() - startUs) / 1000UL));
  core1StageEnd();
}

void blynkSendV4()
{
  core1StageBegin(CORE1_STAGE_BLYNK_TIMER);
  uint32_t perfV4StartUs = micros();

  // v3.9.213: V4 is the lightweight application heartbeat. It may remain
  // random-looking, but it must differ from the previous value: the HTTP GET
  // endpoint exposes no write timestamp, so an equal consecutive value would
  // be indistinguishable from stale cached data.
  // The queue coalesces delayed V4 writes, so congestion cannot create a
  // heartbeat backlog; the next transmitted value is always the newest one.
  if (WiFi.status() != WL_CONNECTED) { core1StageEnd(); return; }
  if (!Blynk.connected()) { core1StageEnd(); return; }
  static uint8_t heartbeat = 0;
  uint8_t nextHeartbeat = (uint8_t)random(1, 10);
  if (nextHeartbeat == heartbeat) {
    nextHeartbeat = (uint8_t)((heartbeat % 9U) + 1U);
  }
  heartbeat = nextHeartbeat;
  safeWriteI(V4, heartbeat);

  perfRecordMax(perfMaxSlowSendMs,
                (uint32_t)((micros() - perfV4StartUs) / 1000UL));
  core1StageEnd();
}

// v3.9.180: V61/V62 remain independent of V4. The coherent database record is
// generated every two seconds and both halves use the same SensorData snapshot.
void blynkSendCoherentSnapshot()
{
  core1StageBegin(CORE1_STAGE_BLYNK_TIMER);
  uint32_t perfSnapStartUs = micros();

  if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
    core1StageEnd();
    return;
  }

  core0PublishConnectionSnapshot(true);

  // Never replace a pair after either half has entered the TX scheduler.
  if (blynkCoherentPairActive) { core1StageEnd(); return; }
  Core1TelemetryPair pair;
  if (telemetryPairQueue == NULL ||
      xQueuePeek(telemetryPairQueue, &pair, 0) != pdTRUE ||
      pair.sequence == 0 || pair.sequence == blynkLastAcceptedTelemetrySequence) {
    core1StageEnd();
    return;
  }

  // Accept only the complete, immutable Core-1 record pair.
  blynkQueueCoherentSnapshot(pair);

  perfRecordMax(perfMaxSlowSendMs,
                (uint32_t)((micros() - perfSnapStartUs) / 1000UL));
  core1StageEnd();
}

void blynkSendSlow()
{
  core1StageBegin(CORE1_STAGE_BLYNK_TIMER);
  uint32_t perfSlowStartUs = micros();

  // Environmental dashboard convenience data — sent every BLYNK_SEND_SLOW_MS (60s)
  if (WiFi.status() != WL_CONNECTED) { core1StageEnd(); return; }
  SensorData snap;
  if (!blynkGetSensorSnapshot(snap)) { core1StageEnd(); return; }

  if (blynkTxCongested()) {
    blynkTxCongestionSkips++;
    core1StageEnd();
    return;
  }

  // V61 carries the complete coherent sensor record. These individual pins
  // are dashboard conveniences and intentionally update less frequently.
  safeWrite (V1,  snap.sound_db);
  safeWrite (V3,  snap.uvi);
  safeWrite (V5,  snap.tvoc_ppb);
  safeWrite (V8,  snap.angle[0]);
  safeWrite (V9,  snap.angle[1]);
  safeWrite (V10, snap.angle[2]);
  safeWrite (V11, snap.imu_temp);
  safeWrite (V18, snap.dust_mg);
  safeWrite (V23, snap.eco2_ppm);
  safeWrite (V24, snap.aqi);
  safeWrite (V25, snap.co_heater_v);
  perfRecordMax(perfMaxSlowSendMs,
                (uint32_t)((micros() - perfSlowStartUs) / 1000UL));
  core1StageEnd();
}

// ─── DIAGNOSTIC VIRTUAL PINS — every 10 seconds ──────────────────────────────
// Internal counters/timers continue constantly. Only their Blynk publication
// is rate-limited to one snapshot every 10 seconds. This prevents diagnostic
// traffic from being added to the 5-second FAST telemetry pathway.
//
// v3.8: V51 is intentionally LATCHED to the previous-run RTC checkpoint.
// rtcReportPreviousCheckpoint() copies the pre-boot RTC record into
// rtcPreviousRunCheckpoint before the new run starts writing checkpoints.
// rtcFormatPreviousRun() therefore always formats that same snapshot.
// Re-publishing V51 every 10 seconds does NOT change its contents.
// V51 remains unchanged for the entire current run and is replaced only after
// the next reboot/restart, when a new previous-run snapshot is captured.
// v3.9.91: diagnostic publication cadence is BLYNK_SEND_DIAG_MS (60s) — see the
// constant's definition above for why. Field list/order unchanged.
void blynkSendDiagnostics()
{
  if (WiFi.status() != WL_CONNECTED) return;
  if (!Blynk.connected()) return;
  if (blynkTxCongested(BLYNK_TX_HARD_LIMIT)) {
    blynkTxHardSkips++;
    return;
  }

  // Diagnostics are lowest priority: never add a large diagnostic burst to an
  // already-congested queue. They are retained maxima/counters, so skipping a
  // cycle loses no historical information.
  // Existing V34 diagnostic channel: ESP32 reset cause from boot.
  safeWriteS(V34, bootResetReasonText);

  // Performance diagnostics V35-V44. These are retained maxima since boot
  // except V35 (count) and V36 (current PPS age).
  safeWriteI(V35, (int)core0SensorDiag.ppsCount);
  safeWriteI(V36, (int)core0SensorDiag.ppsAgeMs);
  safeWriteI(V37, (int)perfMaxLoopGapMs);
  safeWriteI(V38, (int)core0SensorDiag.maxSensorCycleMs);
  safeWriteI(V39, (int)perfMaxBlynkRunMs);
  safeWriteI(V40, (int)core0SensorDiag.maxGpsFeedMs);
  safeWriteI(V41, (int)core0SensorDiag.maxI2cPathMs);
  safeWriteI(V42, (int)perfMaxFastSendMs);
  safeWriteI(V43, (int)perfMaxSlowSendMs);
  safeWriteI(V44, (int)core0SensorDiag.maxPpsLedResponseMs);
  safeWriteI(V45, (int)core0SensorDiag.lastI2cDevice);
  safeWriteI(V46, (int)core0SensorDiag.lastI2cOperation);
  safeWriteI(V47, (int)core0SensorDiag.i2cErrorCount);
  safeWriteI(V48, (int)core0SensorDiag.maxAht21Ms);
  safeWriteI(V49, (int)core0SensorDiag.maxEns160Ms);
  safeWriteI(V50, (int)core0SensorDiag.maxBmi160Ms);
  safeWriteS(V52, core1StageText(core1StageId));
  safeWriteI(V53, (int)core1MaxStageMs);
  safeWriteI(V54, (int)core1StallCount);
  safeWriteI(V56, (int)blynkTxLastVPin);
  safeWriteI(V57, (int)blynkTxLastCallMs);
  safeWriteI(V58, (int)blynkTxMaxCallMs);
  safeWriteI(V59, (int)blynkTxCallCount);
  safeWriteI(V60, (int)blynkTxLastValueLen);
  safeWriteI(V63, (int)blynkTxPendingCount());
  safeWriteI(V64, (int)blynkTxLastVPin);
  safeWriteI(V65, (int)blynkTxMaxCallMs);

  // v3.8: V51 carries the complete PREVIOUS-RUN RTC checkpoint as one
  // latched string. It is deliberately sourced only from
  // rtcPreviousRunCheckpoint, never from the live RTC checkpoint.
  char rtcText[512];
  rtcFormatPreviousRun(rtcText, sizeof(rtcText));
  safeWriteS(V51, rtcText);
}


// v2.5: Blynk Map widget on V33. The Map widget's virtualWrite convention
// is Blynk.virtualWrite(vPin, index, lat, lon, value):
//   • index — identifies/updates a marker; reusing the same index moves an
//     existing marker, a new index drops a new one. We use the GPS UNIX
//     timestamp (sd.gps_unix_time, computed on Core 0 from GPS UTC date/time
//     — see gpsUnixTime()) so every send is naturally a distinct, monotonic,
//     human-meaningful index, and the map accumulates a trail of points over
//     the deployment rather than a single marker that keeps moving. Kept as
//     the GPS timestamp regardless of which position source below is used —
//     it's the only reliable real-world clock in the system either way.
//   • lat/lon — v2.6: source is now selected automatically each send:
//       - If inertial-nav data EXISTS (BMI160 present and inav_state is one
//         of GPS_FIX / IMU_RECENT / IMU_STALE — i.e. an origin has been set
//         from a real GPS fix and the EKF hasn't been forced into LOST),
//         use the fused sd.inav_lat/sd.inav_lng. This keeps the trail going
//         through brief GPS dropouts via BMI160 dead-reckoning.
//       - Otherwise (no BMI160 wired, or inav_state is NO_FIX_YET/LOST —
//         i.e. inertial-nav data does NOT exist/isn't trustworthy right
//         now), fall back to raw GPS sd.lat/sd.lng, gated on sd.gps_fix,
//         exactly as before. This is also the behaviour with no IMU wired
//         at all, since inav_state then never leaves NO_FIX_YET.
//   • value — a fixed label string identifying this device on the map.
// Deliberately its own timer tier (MAP_SEND_INTERVAL_MS, default 60s) rather
// than piggybacking on FAST(5s)/SLOW(10s) — a marker every few seconds would
// flood the Map widget on any deployment longer than a few minutes.
void blynkSendMap()
{
  if (WiFi.status() != WL_CONNECTED) return;
  if (blynkTxCongested()) {
    blynkTxCongestionSkips++;
    return;
  }

  SensorData snap;
  if (!blynkGetSensorSnapshot(snap)) return;
  const bool   gpsFixNow = snap.gps_fix;
  const double gpsLat    = snap.lat;
  const double gpsLng    = snap.lng;
  const int    inavState = snap.inav_state;
  const double inavLat   = snap.inav_lat;
  const double inavLng   = snap.inav_lng;
  const long   unixT     = snap.gps_unix_time;

  if (unixT <= 0) return;   // no valid GPS time yet — skip this cycle, try again later

  // Does inertial-nav data exist and can it be trusted right now? IMU_STALE
  // is deliberately EXCLUDED: it means several consecutive GPS fixes have
  // already been missed, so the EKF is dead-reckoning on pure gyro/velocity
  // integration with no fresh correction. Held velocity + drifting heading
  // (see inavPredict()) traces looping arcs on the map the longer this runs.
  // Only trust INAV while GPS is actively correcting it (GPS_FIX) or the gap
  // is still short (IMU_RECENT); fall back to raw GPS as soon as it goes
  // stale, well before the map-visible drift becomes large.
  bool inavExists = (inavState == INAV_STATE_GPS_FIX ||
                      inavState == INAV_STATE_IMU_RECENT);

  double lat, lng;
  const char* src;
  if (inavExists) {
    lat = inavLat; lng = inavLng; src = "INAV";
  } else if (gpsFixNow) {
    lat = gpsLat; lng = gpsLng; src = "GPS";
  } else {
    return;   // neither source is trustworthy this cycle — skip, try again later
  }

  // Queue the Map update too, so it cannot collide with the FAST/SLOW/
  // diagnostics burst.  Only the newest map point needs to be retained.
  blynkMapPending = true;
  blynkMapIndex = unixT;
  blynkMapLat = lat;
  blynkMapLng = lng;

  if (DEBUGON)
    DBG_PRINTF("[BLYNK MAP] idx=%ld lat=%.6f lng=%.6f src=%s\n", unixT, lat, lng, src);
}

// ─── LEDC cross-core protection ─────────────────────────────────────────────
// v3.9.4 FIX: LEDC is used by BOTH cores. Core 1 drives the non-blocking
// buzzer, while Core 0 drives the CO heater PWM. ESP32 Arduino core 3.x LEDC
// operations enter the ESP-IDF LEDC driver and may briefly hold its internal
// critical section with interrupts disabled. Calling LEDC APIs concurrently
// from the two cores can therefore make one core wait inside the driver long
// enough to trip the Interrupt-WDT. The V55 software-reset boot exposed this
// race immediately because WiFi connection success schedules a buzzer tone
// while the sensor task is already running. Serialize ALL sketch-level LEDC
// operations with a dedicated mutex so only one core can touch the LEDC driver
// at a time. No sensor, buzzer, or PWM logic is removed.
SemaphoreHandle_t ledcMutex = NULL;
volatile uint32_t ledcLockTakeCalls = 0;
volatile uint32_t ledcLockTimeouts = 0;
volatile uint32_t ledcLockMaxWaitUs = 0;

bool ledcTakeLock(uint32_t timeoutMs = 10)
{
  if (ledcMutex == NULL) return false;
  const uint32_t startUs = micros();
  ++ledcLockTakeCalls;
  const bool ok = (xSemaphoreTake(ledcMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
  const uint32_t waitedUs = micros() - startUs;
  if (waitedUs > ledcLockMaxWaitUs) ledcLockMaxWaitUs = waitedUs;
  if (!ok) ++ledcLockTimeouts;
  return ok;
}

void ledcGiveLock()
{
  if (ledcMutex != NULL) xSemaphoreGive(ledcMutex);
}

// Arduino-ESP32 3.x LEDC wrappers.
// Keep the application-level pin-based wrapper names unchanged so the buzzer
// and CO state machines remain byte-for-byte equivalent at their call sites.
// Explicit channels 0 and 8 are retained: on the original ESP32 these are in
// different LEDC groups, preserving independent frequency changes.
bool ledcAttachSafe(uint8_t pin, uint32_t freq, uint8_t resolution)
{
  int channel = (pin == buzzPin) ? LEDC_CHANNEL_BUZZER : LEDC_CHANNEL_CO;
  return ledcAttachChannelSafe(pin, freq, resolution, channel);
}

bool ledcAttachChannelSafe(uint8_t pin, uint32_t freq, uint8_t resolution, int8_t channel)
{
  if (!ledcTakeLock()) return false;
  bool ok = ledcAttachChannel(pin, freq, resolution, channel);
  ledcGiveLock();
  return ok;
}

bool ledcChangeFrequencySafe(uint8_t pin, uint32_t freq, uint8_t resolution)
{
  if (!ledcTakeLock()) return false;
  uint32_t actual = ledcChangeFrequency(pin, freq, resolution);
  ledcGiveLock();
  return actual != 0;
}

bool ledcWriteSafe(uint8_t pin, uint32_t duty)
{
  if (!ledcTakeLock()) return false;
  bool ok = ledcWrite(pin, duty);
  ledcGiveLock();
  return ok;
}

bool ledcDetachSafe(uint8_t pin)
{
  if (!ledcTakeLock()) return false;
  bool ok = ledcDetach(pin);
  ledcGiveLock();
  return ok;
}

// v3.9.23 WiFi fault/event diagnostics and bounded recovery.
volatile uint32_t wifiEventCount = 0;
volatile uint32_t wifiGotIpEventCount = 0;
volatile uint8_t wifiLastDisconnectReason = 0;
volatile uint32_t wifiLastDisconnectEventMs = 0;
// Keep the initiating cause stable for the whole outage. A later failed
// association (commonly reason 4 / ASSOC_EXPIRE) must not erase it.
volatile uint8_t wifiOutageInitialDisconnectReason = 0;
volatile uint32_t wifiOutageInitialDisconnectEventMs = 0;
volatile uint32_t wifiLastGotIpEventMs = 0;
// v3.9.41: set by the WiFi event callback the instant a real
// ARDUINO_EVENT_WIFI_STA_DISCONNECTED fires. Previously wifiService() had no
// event-driven path at all — it only noticed a disconnect by polling
// WiFi.status() on a fixed WIFI_RECONNECT_MS (30s) timer, and WiFi's own
// built-in fast reconnect was deliberately disabled (WiFi.setAutoReconnect
// (false)) to avoid the two reconnect paths fighting each other. That
// combination meant every real disconnect sat idle for up to 30s before
// anything reacted at all, on top of the handshake time after that — this
// flag lets wifiService() react on the very next loop() pass instead.
volatile bool wifiDisconnectEventPending = false;
// v3.9.62: detailed WiFi event attribution diagnostics. These fields let us
// distinguish a disconnect caused by our own recovery path (WiFi.disconnect())
// from a disconnect reported spontaneously by the WiFi driver/AP. The event
// callback only records state; formatted output remains in wifiService().
volatile uint32_t wifiDisconnectEventSequence = 0;
volatile uint32_t wifiUnexpectedDisconnectCount = 0;
volatile uint32_t wifiStaConnectedEventCount = 0;
volatile uint32_t wifiLastStaConnectedEventMs = 0;
volatile bool wifiLastDisconnectWasExpected = false;
volatile uint32_t wifiExpectedDisconnectUntilMs = 0;
// v3.9.63: after an unexpected STA_DISCONNECTED event, do NOT immediately
// call WiFi.disconnect().  That would destroy the original driver/AP state
// before we can see whether the ESP32 naturally reassociates.  Observe the
// connection for a bounded window first; only then fall back to the existing
// clean recovery path.
static bool wifiUnexpectedObservationActive = false;
static uint32_t wifiUnexpectedObservationUntilMs = 0;
static uint32_t wifiUnexpectedObservationStartedMs = 0;
static unsigned long wifiUnexpectedObservationLastLogMs = 0;
#define WIFI_UNEXPECTED_OBSERVATION_MS 15000UL
#define WIFI_FAST_REAUTH_MAX_ATTEMPTS 6U
#define WIFI_STA_INACTIVE_TIME_S 6U
static uint8_t wifiFastReauthAttempts = 0;
static unsigned long wifiFastReauthNextMillis = 0;
static bool wifiFastReauthScheduled = false;
static uint8_t wifiLastGoodBssid[6] = {};
static uint8_t wifiLastGoodChannel = 0;
static bool wifiLastGoodApValid = false;
volatile uint32_t wifiLastEventId = 0;
volatile uint32_t wifiLastEventMs = 0;
volatile uint8_t wifiLastEventReason = 0;
uint32_t wifiConnectAttempts = 0;
uint32_t wifiSuccessfulConnections = 0;
uint32_t wifiConsecutiveFailures = 0;
unsigned long wifiOfflineSince = 0;
unsigned long wifiLastGotIpMillis = 0;
#define WIFI_MAX_CONSECUTIVE_FAILURES 6U
#define WIFI_MAX_OFFLINE_MS 300000UL
#define WIFI_ASSOC_ATTEMPT_TIMEOUT_MS 30000UL
#define WIFI_HARD_REBUILD_AFTER_FAILURES 3U
#define WIFI_FINAL_RECOVERY_RESTART_MS 120000UL
static bool wifiHardRebuildUsedThisOutage = false;

void wifiEventCallback(arduino_event_t *event)
{
  // This callback runs in the WiFi event task, not Arduino loop(). Keep it
  // thread-safe: record only single-word state here; formatted reporting is
  // deferred to loop()/wifiService().
  if (!event) return;

  const uint32_t now = millis();
  ++wifiEventCount;
  wifiLastEventId = (uint32_t)event->event_id;
  wifiLastEventMs = now;

  if (event->event_id == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wifiLastDisconnectReason = event->event_info.wifi_sta_disconnected.reason;
    wifiLastEventReason = wifiLastDisconnectReason;
    wifiLastDisconnectEventMs = now;
    const bool expected = (wifiExpectedDisconnectUntilMs != 0 &&
                           (int32_t)(wifiExpectedDisconnectUntilMs - now) >= 0);
    wifiLastDisconnectWasExpected = expected;
    if (!expected) {
      ++wifiUnexpectedDisconnectCount;
      if (wifiOutageInitialDisconnectReason == 0) {
        wifiOutageInitialDisconnectReason = wifiLastDisconnectReason;
        wifiOutageInitialDisconnectEventMs = now;
      }
    }
    ++wifiDisconnectEventSequence;
    wifiDisconnectEventPending = true;
  } else if (event->event_id == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    // STA_CONNECTED is deliberately recorded separately from GOT_IP.  This
    // tells us whether the station actually reassociated before DHCP/IP
    // completion, which is important for diagnosing reason=8 disconnects.
    ++wifiStaConnectedEventCount;
    wifiLastStaConnectedEventMs = now;
  } else if (event->event_id == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    ++wifiGotIpEventCount;
    wifiLastGotIpEventMs = now;
  }
}

bool wifiConnectInProgress = false;
unsigned long wifiConnectStarted = 0;
unsigned long wifiFailureAlertAt = 0;
// v3.9.14 FIX: WiFi.disconnect() is asynchronous — the STA driver has not
// necessarily finished tearing down by the time it returns. Calling
// WiFi.begin() in the very same statement/loop-pass (as every reconnect path
// used to) can hit the driver mid-teardown and produce
// "wifi:sta is connecting, cannot set config", silently dropping that begin()
// attempt. The device then sits doing nothing until the NEXT scheduled retry,
// which is most of where "why does it take so long to reconnect" comes from.
// wifiDisconnectSettleUntil gives the driver a short, bounded, non-blocking
// window to finish disconnecting before wifiService() is allowed to call
// WiFi.begin() again.
unsigned long wifiDisconnectSettleUntil = 0;
#define WIFI_DISCONNECT_SETTLE_MS 2000UL
// Set alongside wifiDisconnectSettleUntil whenever a disconnect was issued
// with the intent to reconnect once settled. Kept separate from
// lastWiFiCheck's periodic 30s gate so the settled reconnect fires on the
// very next loop pass instead of waiting for the next periodic window.
bool wifiReconnectPending = false;
// Blynk connection is deliberately managed separately from WiFi.
// Blynk.run() must NEVER be allowed to initiate an unbounded connection
// attempt on Core 1, because that can starve the ESP32 interrupt watchdog
// when the Blynk server is unreachable.  We therefore make a bounded TCP
// probe first, use a short 1s Blynk I/O timeout, and service
// the same handshake incrementally until it completes.
// v3.9.42 DIAGNOSTIC ONLY — added to actually see what state the connection
// is in during a silent stall (no reconnect logged, no data flowing). Prints
// on a short fixed interval; takes no action and touches no connection state.
// This must stay purely observational: no thresholds here may ever set
// blynkForceReconnect or call disconnect() — see v3.9.40/41/42 history above
// for why an earlier watchdog that DID act on a similar signal caused
// self-inflicted disconnects. Placed here (after blynkHandshakeActive and
// blynkForceReconnect are declared) rather than near blynkTxService(),
// because both are needed and moving the variables would touch working code.
#define BLYNK_STATUS_LOG_MS 5000UL
static unsigned long lastBlynkStatusLog = 0;
static void blynkStatusLogPassive()
{
  const unsigned long now = millis();
  if (now - lastBlynkStatusLog < BLYNK_STATUS_LOG_MS) return;
  lastBlynkStatusLog = now;

  DBG_PRINTF("[Blynk STATUS] connected=%d wifiStatus=%d RSSI=%d rxAge=%lums txPending=%u lastTxAttemptAge=%lums lastTxSuccessAge=%lums handshakeActive=%d forceReconnect=%d\n",
                Blynk.connected() ? 1 : 0,
                (int)WiFi.status(),
                WiFi.RSSI(),
                (unsigned long)(blynkLastRxMillis ? now - blynkLastRxMillis : 0),
                (unsigned)blynkTxPendingCount(),
                (unsigned long)(blynkLastTxAttemptMillis ? now - blynkLastTxAttemptMillis : 0),
                (unsigned long)(blynkLastTxSuccessMillis ? now - blynkLastTxSuccessMillis : 0),
                blynkHandshakeActive ? 1 : 0,
                blynkForceReconnect ? 1 : 0);
}
// v3.9.66 DIAGNOSTIC ONLY — capture the transition from a healthy Blynk/TCP
// session to an RX-stale session without taking any recovery action.
// This deliberately does NOT set blynkForceReconnect, disconnect anything,
// change timers, or alter recovery thresholds. It only reports the live state
// once RX has been silent for 10s, then every 2s while the silence continues.
// The goal is to catch whether Blynk.run() is still being serviced, whether
// the raw transport read/write sides are still active, and whether TX backlog
// grows while RX stops progressing. Serial.printf() is used intentionally so
// DEBUGON=OFF does not hide the diagnostic.
#define BLYNK_RX_STALL_DIAG_START_MS 10000UL
#define BLYNK_RX_STALL_DIAG_REPEAT_MS 2000UL
static unsigned long lastBlynkRxStallDiag = 0;
static bool blynkRxStallDiagActive = false;
// v3.9.67 compile-order fix: these probe-state variables are declared here,
// before blynkRxStallDiagnosticPassive(), because that diagnostic reports them.
static uint8_t blynkProbeFailCount = 0;
// v3.9.24: true while a non-blocking TCP probe (blynkProbeStart/Poll) is
// in progress across one or more loop() passes.
static bool blynkProbeInFlight = false;

static void blynkRxStallDiagnosticPassive()
{
  const unsigned long now = millis();
  const unsigned long rxAge = blynkStockRxProxyAgeMs(now);

  if (!Blynk.connected() || rxAge < BLYNK_RX_STALL_DIAG_START_MS) {
    blynkRxStallDiagActive = false;
    return;
  }

  if (!blynkRxStallDiagActive) {
    blynkRxStallDiagActive = true;
    lastBlynkRxStallDiag = 0;
  }

  if (lastBlynkRxStallDiag != 0 &&
      (now - lastBlynkRxStallDiag) < BLYNK_RX_STALL_DIAG_REPEAT_MS) {
    return;
  }
  lastBlynkRxStallDiag = now;

  Serial.printf("[Blynk RX STALL / STOCK] rxAge=%lums connected=%d wifiStatus=%d RSSI=%d txPending=%u txAttemptAge=%lums txSuccessAge=%lums runMax=%lums runSlow=%lu readCalls=%lu readLast=%lums readMax=%lums writeCalls=%lu writeLast=%lums writeMax=%lums EAGAIN=%lu writeTimeouts=%lu socketErrors=%lu lastErr=%d lastSOERR=%d handshake=%d forceReconnect=%d probe=%d probeFails=%u handshakeFails=%u heap=%u minHeap=%u\n",
                rxAge,
                Blynk.connected() ? 1 : 0,
                (int)WiFi.status(),
                WiFi.RSSI(),
                (unsigned)blynkTxPendingCount(),
                (unsigned long)(blynkLastTxAttemptMillis ? now - blynkLastTxAttemptMillis : 0),
                (unsigned long)(blynkLastTxSuccessMillis ? now - blynkLastTxSuccessMillis : 0),
                (unsigned long)perfMaxBlynkRunMs,
                (unsigned long)diagBlynkRunSlow,
                (unsigned long)blynkTransportReadCalls,
                (unsigned long)blynkTransportReadLastMs,
                (unsigned long)blynkTransportReadMaxMs,
                (unsigned long)blynkTransportWriteCalls,
                (unsigned long)blynkTransportWriteLastMs,
                (unsigned long)blynkTransportWriteMaxMs,
                (unsigned long)blynkTransportWriteEagain,
                (unsigned long)blynkTransportWriteTimeouts,
                (unsigned long)blynkTransportSocketErrors,
                (int)blynkTransportLastErrno,
                (int)blynkTransportLastSoError,
                blynkHandshakeActive ? 1 : 0,
                blynkForceReconnect ? 1 : 0,
                blynkProbeInFlight ? 1 : 0,
                (unsigned)blynkProbeFailCount,
                (unsigned)blynkHandshakeFailCount,
                ESP.getFreeHeap(),
                ESP.getMinFreeHeap());

  // v3.9.67: inspect the underlying TCP socket only while an RX stall is
  // already confirmed by the existing passive diagnostic. This call is
  // observational only and does not consume socket data or trigger recovery.
  _stockBlynkTransportDiagnostics.diagnosticRawRxState();
}

// v3.9.87: the raw-socket snapshot code is retained only as historical diagnostic code; the active transport is stock WiFiClient.
// This is observational only and never consumes Blynk bytes.
#define BLYNK_RX_QUEUE_TRACE_INTERVAL_MS 2000UL
static unsigned long lastBlynkRxQueueTrace = 0;
static void blynkRxQueueTraceService()
{
  const unsigned long now = millis();
  const unsigned long rxAge = blynkStockRxProxyAgeMs(now);

  if (!Blynk.connected() || rxAge < BLYNK_RX_STALL_DIAG_START_MS) {
    lastBlynkRxQueueTrace = 0;
    return;
  }
  if (lastBlynkRxQueueTrace != 0 &&
      (now - lastBlynkRxQueueTrace) < BLYNK_RX_QUEUE_TRACE_INTERVAL_MS) {
    return;
  }
  lastBlynkRxQueueTrace = now;

  Serial.printf("[Blynk RX QUEUE TRACE / STOCK] rxAge=%lums recvCalls=%lu lastRecvAge=%lums hbCount=%lu txPending=%u wifi=%d RSSI=%d\n",
                rxAge,
                (unsigned long)blynkTransportReadCalls,
                (unsigned long)(blynkLastRxMillis ? now - blynkLastRxMillis : 0UL),
                (unsigned long)blynkForcePingCount,
                (unsigned)blynkTxPendingCount(),
                (int)WiFi.status(), WiFi.RSSI());

  _stockBlynkTransportDiagnostics.diagnosticRawRxState();
}

// v3.9.81: periodic connected-session RX trace.
#define BLYNK_RX_TRACE_INTERVAL_MS 5000UL
static unsigned long lastBlynkRxTrace = 0;
static bool blynkRxTraceWasConnected = false;
static void blynkRxTraceService()
{
  const unsigned long now = millis();
  const bool connected = Blynk.connected();
  if (!connected) {
    blynkRxTraceWasConnected = false;
    return;
  }
  if (!blynkRxTraceWasConnected) {
    blynkRxTraceWasConnected = true;
    lastBlynkRxTrace = now;
    _stockBlynkTransportDiagnostics.diagnosticRxTrace("CONNECTED_START");
    return;
  }
  if (now - lastBlynkRxTrace >= BLYNK_RX_TRACE_INTERVAL_MS) {
    lastBlynkRxTrace = now;
    _stockBlynkTransportDiagnostics.diagnosticRxTrace("PERIODIC");
  }
}

// v3.9.80: active recovery for an ESTABLISHED Blynk session whose RX side
// has stopped progressing while WiFi and TCP still look healthy. RX silence
// can be benign for shorter periods, so only 30s triggers recovery.
#define BLYNK_RX_STALL_RECOVERY_MS 30000UL
#define BLYNK_RX_STALL_RECOVERY_HOLDOFF_MS 60000UL
static unsigned long blynkRxStallRecoveryLastMs = 0;

static void blynkRxStallRecoveryService()
{
#if CONNECTION_MODE == 1
  // v3.9.87: do NOT perform the old 30s RX-silence recovery in the stock
  // transport experiment. The stock WiFiClient path does not expose the raw
  // transport's exact byte-receipt timestamp, and a quiet Blynk session is not
  // by itself proof of a dead connection. Let stock BlynkProtocol heartbeat
  // handling decide when the session is actually timed out.
  return;
#else
  const unsigned long now = millis();
  const unsigned long rxAge =
      blynkLastRxMillis ? (unsigned long)(now - blynkLastRxMillis) : 0UL;

  if (!Blynk.connected() || blynkHandshakeActive || WiFi.status() != WL_CONNECTED ||
      rxAge < BLYNK_RX_STALL_RECOVERY_MS) {
    return;
  }

  if (blynkRxStallRecoveryLastMs != 0 &&
      (now - blynkRxStallRecoveryLastMs) < BLYNK_RX_STALL_RECOVERY_HOLDOFF_MS) {
    return;
  }

  Serial.printf("[Blynk RX RECOVERY] 30s established-session RX stall — Blynk-only transport reset rxAge=%lums wifiStatus=%d RSSI=%d txPending=%u handshake=%d forceReconnect=%d probe=%d socketErr=%lu soErr=%d heap=%u minHeap=%u\n",
                rxAge, (int)WiFi.status(), WiFi.RSSI(),
                (unsigned)blynkTxPendingCount(), blynkHandshakeActive ? 1 : 0,
                blynkForceReconnect ? 1 : 0, blynkProbeInFlight ? 1 : 0,
                (unsigned long)blynkTransportSocketErrors,
                (int)blynkTransportLastSoError, ESP.getFreeHeap(), ESP.getMinFreeHeap());

  blynkRxStallRecoveryLastMs = now;

  // Close both Blynk layers. Do NOT touch WiFi here.
  Blynk.disconnect();
#if CONNECTION_MODE == 0
  _stockBlynkTransportDiagnostics.disconnect();
#endif

  blynkHandshakeActive = false;
  blynkForceReconnect = false;
  blynkProbeInFlight = false;
  blynkProbeFailCount = 0;
  blynkHandshakeFailCount = 0;
  blynkConfigured = true;
  blynkOfflineSince = now;
  lastBlynkConnectAttempt = now;
  blynkConnectImmediate = true;

  Serial.println("[Blynk RX RECOVERY] transport closed; WiFi left untouched; fresh bounded TCP preflight requested");
#endif
}

// v3.9.42: the v3.9.40/v3.9.41 RX-staleness watchdog (BLYNK_RX_STALE_MS) was
// removed — see blynkHeartbeatService() for why. blynkForceReconnect is now
// set only by a genuine WiFi re-association (wifiMarkConnected()) or by the
// pre-existing probe/handshake failure-count escalation below, both of which
// reflect an actual connectivity problem rather than a heuristic guess.
// v3.9.39: true after a real WiFi re-association until a NEW Blynk protocol
// connection is confirmed.  This deliberately bypasses Blynk.connected() as a
// gate because a stale Blynk 0.6.1 protocol object can report connected after
// the underlying WiFi/TCP session has died.  In that state the old v3.9.38
// reconnect path could repeatedly print "handshake pending" but never enter
// the TCP probe because !Blynk.connected() was false.
#define BLYNK_SOFT_RESET_HOLDOFF_MS 15000UL
// v3.9.43 FIX: BLYNK_CONNECT_RETRY_MS/BLYNK_HANDSHAKE_TIMEOUT_MS/
// BLYNK_SERVER_PROBE_TIMEOUT_MS/BLYNK_HANDSHAKE_FAILS_BEFORE_WIFI_RESET/
// BLYNK_WIFI_RECOVERY_CYCLE_THRESHOLD together determine total outage
// time after a genuine dead-socket stall (confirmed by field log: write
// timeout errno=11/EAGAIN after 121s connected, TX buffer stopped draining —
// this is a real peer-side stall, not a self-inflicted disconnect). The
// original values (10s retry + 10s handshake + 6s probe, x3 handshake fails,
// x3 WiFi-reset fails) added up to ~2m38s end-to-end before a software
// restart, confirmed against the same field log's timestamps. That is far
// too slow for a platform whose position/readings are moving in real time.
// The multi-tier structure itself (probe -> handshake-fail count -> WiFi
// reset -> bounded retry cycle) is intentionally kept — see the v3.9.9/v3.9.10 notes
// below explaining why a single fast reconnect attempt is not enough on its
// own (a reachable TCP port does not guarantee an immediate Blynk login
// response). Only the per-stage durations and fail counts are tightened:
//   probe 6s -> 4s, handshake 10s -> 6s, retry gate 10s -> 4s,
//   handshake fails before WiFi reset 3 -> 2, WiFi-recovery cycle 3 -> 2.
// New worst case: ~2 x (4s probe + 4s retry + 6s handshake) per tier x 2
// tiers =~ 55-60s to a new bounded recovery cycle. Typical case
// (server responds within one probe/handshake cycle) recovers in well under
// 15s, matching the WiFi-layer speed already achieved in v3.9.41.
#define BLYNK_CONNECT_RETRY_MS 4000UL
#define BLYNK_CONNECT_TIMEOUT_S 5UL
#define BLYNK_HANDSHAKE_TIMEOUT_MS 6000UL
#define BLYNK_SERVER_PROBE_TIMEOUT_MS 4000UL
// v3.9.8: if the Blynk server becomes unreachable while WiFi still reports
// connected, do not remain trapped in repeated TCP-probe failures. After a
// small number of consecutive failures, explicitly tear down the Blynk socket
// and restart the WiFi association. This gives the network stack a clean
// recovery path without requiring a physical ESP32 reset.
// v3.9.24: raised from 3U to 6U so a couple of transient probe failures
// (e.g. one dropped ARP/TCP attempt) do not immediately force a full WiFi
// reassociation — only sustained unreachability does.
#define BLYNK_PROBE_FAILS_BEFORE_WIFI_RESET 6U

    // connection attempt right after WiFi associates, when ARP for the server's
    // IP has not yet been resolved. The probe remains hard-bounded by
    // select() rather than an uncertain WiFiClient timeout, there's no
    // watchdog risk in a longer bound — 1s comfortably covers ARP + local-LAN
    // TCP handshake while still being a small fraction of the 10s task-WDT
    // timeout and the 10s BLYNK_CONNECT_RETRY_MS retry interval.

// v3.9.64 WIFI TEARDOWN ATTRIBUTION — diagnostic only.
// Every deliberate WiFi.disconnect() is routed through this helper so the
// serial log identifies exactly which firmware path initiated the teardown.
// Recovery behaviour is intentionally unchanged: this helper only records
// state, marks the resulting disconnect event as expected, and performs the
// same WiFi.disconnect() call that the original code performed.
static uint32_t wifiIntentionalTeardownCount = 0;
static char wifiLastTeardownReason[48] = "none";
static uint32_t wifiLastTeardownMillis = 0;

static void wifiIntentionalTeardown(const char *reason, bool powerOff = true, bool eraseAp = false)
{
  const uint32_t now = millis();

  // v3.9.165: automatic reconnect owns only spontaneous AP/security losses.
  // Disable it before an intentional operation so it cannot race this state
  // machine's bounded settle and subsequent WiFi.begin().
  WiFi.setAutoReconnect(false);

  if (wifiIntentionalTeardownCount < 0xFFFFFFFFUL) {
    ++wifiIntentionalTeardownCount;
  }
  snprintf(wifiLastTeardownReason, sizeof(wifiLastTeardownReason),
           "%s", reason ? reason : "UNKNOWN");

  wifiLastTeardownMillis = now;

  const unsigned long rxAge =
      blynkLastRxMillis ? (unsigned long)(now - blynkLastRxMillis) : 0;
  const unsigned long connAge =
      blynkLastConnectedMillis ? (unsigned long)(now - blynkLastConnectedMillis) : 0;

  Serial.printf("[WiFi TEARDOWN] #%lu reason=%s mode=%s millis=%lu status=%d Blynk=%d configured=%d handshake=%d forceReconnect=%d probeInFlight=%d probeFails=%u handshakeFails=%u wifiRecoveryFails=%u rxAge=%lums connAge=%lums txPending=%u RSSI=%d channel=%d BSSID=%s IP=%s GW=%s lastDiscReason=%u eventAge=%lums unexpected=%lu heap=%u minHeap=%u\n",
             (unsigned long)wifiIntentionalTeardownCount,
             wifiLastTeardownReason,
             powerOff ? "RADIO_OFF" : "SOFT_STA",
             (unsigned long)now,
             (int)WiFi.status(),
             Blynk.connected() ? 1 : 0,
             blynkConfigured ? 1 : 0,
             blynkHandshakeActive ? 1 : 0,
             blynkForceReconnect ? 1 : 0,
             blynkProbeInFlight ? 1 : 0,
             (unsigned)blynkProbeFailCount,
             (unsigned)blynkHandshakeFailCount,
             (unsigned)blynkWifiRecoveryFailCount,
             rxAge,
             connAge,
             (unsigned)blynkTxPendingCount(),
             WiFi.RSSI(),
             WiFi.channel(),
             WiFi.BSSIDstr().c_str(),
             WiFi.localIP().toString().c_str(),
             WiFi.gatewayIP().toString().c_str(),
             (unsigned)wifiLastDisconnectReason,
             (unsigned long)(wifiLastDisconnectEventMs ? now - wifiLastDisconnectEventMs : 0),
             (unsigned long)wifiUnexpectedDisconnectCount,
             ESP.getFreeHeap(),
             ESP.getMinFreeHeap());

  // Mark this disconnect as intentional before invoking the asynchronous
  // driver call so the WiFi event callback classifies reason=8 correctly.
  wifiExpectedDisconnectUntilMs = now + 2000UL;

  // Generic WiFi failures keep the established hard-recovery behavior.
  // v3.9.149's Blynk TCP-stack escalation deliberately uses the soft form:
  // the v3.9.148 field run proved that powering the radio off for a Blynk-only
  // fault produced a status=6/reason=2 association loop on this ESP32.
  // Arduino-ESP32's two-argument overload waits only its short default 100ms.
  // Give the asynchronous driver a bounded second to report disconnection
  // before a later WiFi.begin(). A rare outage may block this communications
  // task for <=1s, but it prevents the much longer "cannot set config" race.
  const bool disconnectOk = WiFi.disconnect(powerOff, eraseAp, 1000UL);
  if (powerOff) {
    // If the station was already logically disconnected, STA.disconnect()
    // can return false before its wrapper reaches STA.end(). mode(WIFI_OFF)
    // makes the requested driver reset unconditional in that case too.
    WiFi.mode(WIFI_OFF);
  }
  Serial.printf("[WiFi TEARDOWN] driverComplete=%d powerOff=%d eraseAP=%d statusAfter=%d\n",
                disconnectOk ? 1 : 0, powerOff ? 1 : 0,
                eraseAp ? 1 : 0, (int)WiFi.status());
}

static void blynkSequenceHoleRotateSocket(const char *source)
{
  if (blynkSequenceHoleSocketTrialCount < 255U) {
    ++blynkSequenceHoleSocketTrialCount;
  }
  blynkSequenceHoleRecoveryTrial = true;
  blynkSequenceHoleApplicationCanary = false;
  blynkApplicationCanaryPassed = 0;
  blynkApplicationCanarySentBaseline = 0;
  blynkApplicationCanaryVerifiedBaseline = 0;
  blynkSequenceHoleRecoveryStartedMillis = millis();

  Serial.printf("[Blynk TCP SEQUENCE HOLE] source=%s; rotating owner-managed socket, heartbeat-only trial %u/%u; WiFi untouched\n",
                source ? source : "UNKNOWN",
                (unsigned)blynkSequenceHoleSocketTrialCount,
                (unsigned)BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS);
  blynkStockProtocolReset();
  blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
  blynkStockStateSinceMillis = millis();
  blynkOfflineSince = millis();
  blynkStockNextRunMillis = millis() + BLYNK_STOCK_RETRY_INTERVAL_MS;
  blynkTxDropNonPriority();
  blynkTxDropCoherentPair();
}

static void blynkSequenceHoleEnterServerCooldown(const char *source)
{
  const uint32_t now = millis();
  if (blynkSequenceHoleServerCooldownCount < 0xFFFFFFFFUL) {
    ++blynkSequenceHoleServerCooldownCount;
  }

  Serial.printf("[Blynk SERVER COOLDOWN] source=%s three replacement sessions remained one-way; protocol-only reset, WiFi/MCU preserved, retry in %lums cooldowns=%lu\n",
                source ? source : "UNKNOWN",
                (unsigned long)BLYNK_SEQUENCE_HOLE_SERVER_COOLDOWN_MS,
                (unsigned long)blynkSequenceHoleServerCooldownCount);

  blynkStockProtocolReset();
  blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
  blynkStockStateSinceMillis = now;
  blynkOfflineSince = now;
  blynkStockNextRunMillis = now + BLYNK_SEQUENCE_HOLE_SERVER_COOLDOWN_MS;
  blynkStockTxRecoveryPending = false;
  blynkSequenceHoleRecoveryTrial = true;
  blynkSequenceHoleApplicationCanary = false;
  blynkApplicationCanaryPassed = 0;
  blynkApplicationCanarySentBaseline = 0;
  blynkApplicationCanaryVerifiedBaseline = 0;
  blynkSequenceHoleRecoveryStartedMillis = now;
  blynkTxDropNonPriority();
  blynkTxDropCoherentPair();
}

// The field evidence establishes the required reset domain. After a proven
// one-way established stream, two new Blynk TCP sockets also receive no SYN
// result for their complete one-second budgets. At that point HTTP and Blynk
// have both failed while STA remains associated. WiFi.disconnect()/mode-off
// was tried twice and both times association stuck at reason 2 until the MCU
// reset. Do not repeat that ineffective transition; reset the complete WiFi +
// lwIP state directly. Immediate refusals and ordinary server/login failures
// never enter this path.
static void blynkSequenceHoleRequestBoundedRestart(const char *source)
{
  if (blynkStockWifiStackRecoveryAttempted ||
      blynkStockWifiStackRecoveryInProgress ||
      sharedPathRecoveryState != SHARED_PATH_IDLE) return;

  blynkStockWifiStackRecoveryAttempted = true;
  blynkStockWifiStackRecoveryInProgress = true;
  if (blynkStockWifiStackRecoveryCount < 0xFFFFFFFFUL) {
    ++blynkStockWifiStackRecoveryCount;
  }
  telemetryHttpPauseRequested = true;

  Serial.printf("[Blynk SHARED PATH RECOVERY] source=%s generations=%u tcpFailures=%u tcpTimeouts=%u httpFailures=%lu; proven shared lwIP path failure, restarting complete MCU (WiFi teardown bypassed)\n",
                source ? source : "UNKNOWN",
                (unsigned)blynkSequenceHoleRecentGenerations,
                (unsigned)blynkStockOutageTcpConnectFailures,
                (unsigned)blynkStockOutageTcpConnectTimeouts,
                (unsigned long)telemetryHttpFailureCount);
  diagnosticResetRequest("PROVEN_SHARED_TCP_PATH");
}
// Returns true while this state machine owns the current wifiService() pass.
static bool blynkSharedPathRecoveryService(uint32_t now)
{
  if (sharedPathRecoveryState == SHARED_PATH_IDLE) return false;

  if (sharedPathRecoveryState == SHARED_PATH_QUIESCE) {
    if (now - sharedPathRecoveryStateAtMs < SHARED_PATH_QUIESCE_MS) return true;
    sharedPathForceFullScan = true;
    wifiIntentionalTeardown("BLYNK_SHARED_PATH_REBUILD", true, true);
    wifiHardRebuildUsedThisOutage = true;
    wifiDisconnectSettleUntil = now + SHARED_PATH_DRIVER_SETTLE_MS;
    wifiReconnectPending = true;
    sharedPathRecoveryState = SHARED_PATH_WAIT_GOT_IP;
    sharedPathRecoveryStateAtMs = now;
    Serial.printf("[Blynk SHARED PATH RECOVERY] WiFi driver stopped; settle=%lums, awaiting fresh GOT_IP\n",
                  (unsigned long)SHARED_PATH_DRIVER_SETTLE_MS);
    return true;
  }

  if (sharedPathRecoveryState == SHARED_PATH_WAIT_GOT_IP) {
    const bool freshGotIp = wifiLastGotIpEventMs != 0 &&
        (int32_t)(wifiLastGotIpEventMs - sharedPathRecoveryStateAtMs) > 0;
    if (WiFi.status() == WL_CONNECTED && freshGotIp) {
      wifiMarkConnected();
      sharedPathRecoveryState = SHARED_PATH_POST_IP_GRACE;
      sharedPathRecoveryStateAtMs = now;
      Serial.printf("[Blynk SHARED PATH RECOVERY] fresh GOT_IP accepted; HTTP remains paused for %lums\n",
                    (unsigned long)SHARED_PATH_POST_IP_GRACE_MS);
      return true;
    }
    // Suppress the stale-WL_CONNECTED shortcut, but allow the normal WiFi
    // association/timeout machinery below to run while genuinely disconnected.
    return WiFi.status() == WL_CONNECTED;
  }

  if (now - sharedPathRecoveryStateAtMs < SHARED_PATH_POST_IP_GRACE_MS) return true;
  sharedPathRecoveryState = SHARED_PATH_IDLE;
  sharedPathRecoveryStateAtMs = now;
  blynkStockWifiStackRecoveryInProgress = false;
  telemetryHttpPauseRequested = false;
  Serial.println("[Blynk SHARED PATH RECOVERY] post-IP grace complete; bulk HTTP may resume");
  return false;
}

void wifiStartConnect()
{
  ++wifiConnectAttempts;
  DBG_PRINTF("[WiFi] Connecting to %s... attempt=%lu consecutiveFailures=%lu\n",
             WIFI_SSID, (unsigned long)wifiConnectAttempts,
             (unsigned long)wifiConsecutiveFailures);
  // v3.9.80: WiFi was explicitly powered off by the preceding intentional
  // teardown. Re-enable STA only after wifiService() has observed the settle
  // interval, so WiFi.begin() never races the asynchronous teardown.
  WiFi.mode(WIFI_STA);
  yield();
  const esp_err_t inactiveTimeResult =
      esp_wifi_set_inactive_time(WIFI_IF_STA, WIFI_STA_INACTIVE_TIME_S);
#if WIFI_TEST_LEGACY_BG
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
  DBG_PRINTLN("[WiFi TEST] STA restricted to 802.11b/g (no HT/A-MPDU)");
#else
  const esp_err_t wifiProtocolResult = esp_wifi_set_protocol(
      WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  DBG_PRINTF("[WiFi] STA protocol restored to 802.11b/g/n (HT/A-MPDU available), result=%d\n",
             (int)wifiProtocolResult);
#endif
  // v3.9.41 FIX: was forced to max (WIFI_POWER_19_5dBm) on every reconnect.
  // Max TX power on a nearby/uncongested AP can itself cause instability —
  // an overly "loud" client can trigger AP-side retransmit storms or
  // outright association drops (WIFI_REASON_ASSOC_LEAVE and similar), which
  // looks identical to "the ESP32 keeps randomly disconnecting" from this
  // side. A stable ESP8266 on the same network typically runs at a much
  // lower default TX power, which is a plausible reason it doesn't show this
  // symptom. Backed off to a moderate default; raise it again only if RSSI
  // logs (already printed at every reconnect, see wifiService()) show the
  // link is actually marginal.
  // v3.9.142: back to 19.5 dBm. tcpdump on the server showed Pi->ESP32 traffic
  // fine while ESP32->Pi packets vanished for minutes (uplink-only loss) at
  // RSSI -67 dBm; 15 dBm weakens exactly that direction.
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  // v3.9.26: DISABLED. WiFi.setAutoReconnect(true) makes the underlying
  // ESP32 WiFi driver run its OWN independent reconnect logic on every
  // disconnect, entirely separate from and unaware of wifiService()'s
  // custom reconnect state machine below. Two reconnect mechanisms acting
  // on the same radio concurrently is a well-documented cause of repeated
  // WIFI_REASON_ASSOC_LEAVE (reason 8) churn: one path can tear down or
  // reassociate mid-way through what the other just established, which
  // matches the "eventCount=13 for only 3 real reconnect attempts" pattern
  // observed in production logs, correlated with Blynk handshake timeouts
  // (a TCP session invalidated mid-handshake by a silent reassociation
  // looks identical to a genuinely slow/hung server). wifiService()
  // already owns retry timing/backoff explicitly — let it be the only
  // thing that decides when to reconnect.
  // v3.9.174: wifiService() is the single reconnect owner.  Enabling Arduino's
  // automatic reconnect here creates a second owner which can issue its own
  // disconnect/connect while the explicit observation/recovery state machine
  // is active.  The resulting local STA_LEAVING events invalidate live TCP.
  WiFi.setAutoReconnect(false);
  // Avoid a full-channel scan on the first recovery attempts. Fall back to an
  // unconstrained scan after three failures so an AP channel/BSSID change can
  // still recover without manual intervention.
  // A spontaneous AP/security loss invalidates assumptions about both channel
  // and BSSID. Pinning recovery to the old AP can turn a channel change or
  // roaming event into three 20-second failures before a full scan is tried.
  const bool useCachedAp = !sharedPathForceFullScan && wifiLastGoodApValid &&
      wifiSuccessfulConnections > 0 && wifiConsecutiveFailures < 3U &&
      wifiOutageInitialDisconnectReason == 0;
  wl_status_t beginStatus;
  if (useCachedAp) {
    beginStatus = WiFi.begin(WIFI_SSID, WIFI_PASS,
                             (int32_t)wifiLastGoodChannel,
                             wifiLastGoodBssid, true);
  } else {
    beginStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  Serial.printf("[WiFi CONNECT POLICY] cachedAP=%d channel=%u beginStatus=%d inactive=%us setResult=%d initialReason=%u latestReason=%u\n",
                useCachedAp ? 1 : 0, (unsigned)wifiLastGoodChannel,
                (int)beginStatus, (unsigned)WIFI_STA_INACTIVE_TIME_S,
                (int)inactiveTimeResult,
                (unsigned)wifiOutageInitialDisconnectReason,
                (unsigned)wifiLastDisconnectReason);
  // v3.9.15 FIX — actual root cause of the whole reconnect saga: ESP32 WiFi
  // defaults to modem-sleep (power-save) mode. The radio periodically powers
  // down between AP beacon intervals to save power, which on a self-hosted/
  // LAN server adds intermittent multi-second latency spikes and outright
  // missed packets — exactly matching everything observed across this
  // debugging session: a long, healthy connection that suddenly stalls,
  // TCP probes that fail 3x in a row even though WiFi.status() still claims
  // WL_CONNECTED (association survives modem sleep; low-latency reachability
  // does not), and one-off multi-second blocking spikes that showed up in
  // unrelated places (GPS serial reads, I2C, virtualWrite) because the radio
  // waking from sleep causes brief system-wide timing/current disturbances,
  // not because any one of those functions was itself the culprit. This is
  // the single most common, well-documented cause of "ESP32 WiFi randomly
  // disconnects/latency spikes after a few minutes" reports and is fixed by
  // keeping the radio fully powered instead of letting it cycle into sleep.
  wifiConnectStarted = millis();
  wifiConnectInProgress = true;
}

void wifiMarkConnected()
{
  // v3.9.143: report the REAL driver state (the INIT line only prints the
  // WIFI_SLEEP constant), plus channel/BSSID for correlating with AP logs.
  {
    // v3.9.166: WIFI_SLEEP is the single authority. Previously this location
    // always forced WIFI_PS_NONE and silently defeated WIFI_SLEEP=true.
    // Reapply the selected mode because a driver restart can restore defaults.
    const wifi_ps_type_t requestedPs = WIFI_SLEEP
        ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE;
    const esp_err_t psSetResult = esp_wifi_set_ps(requestedPs);
    wifi_ps_type_t psNow = WIFI_PS_MAX_MODEM;
    const esp_err_t psGetResult = esp_wifi_get_ps(&psNow);
    Serial.printf("[WiFi POWER SAVE VERIFY] requested=%d set=%d get=%d actual=%d (0=NONE,1=MIN_MODEM)\n",
                  (int)requestedPs, (int)psSetResult, (int)psGetResult, (int)psNow);
    wifi_ap_record_t apInfo;
    if (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK) {
      memcpy(wifiLastGoodBssid, apInfo.bssid, sizeof(wifiLastGoodBssid));
      wifiLastGoodChannel = apInfo.primary;
      wifiLastGoodApValid = true;
      DBG_PRINTF("[WiFi REAL] ps=%d (0=NONE) ch=%u rssi=%d bssid=%02X:%02X:%02X:%02X:%02X:%02X\n",
                 (int)psNow, (unsigned)apInfo.primary, (int)apInfo.rssi,
                 apInfo.bssid[0], apInfo.bssid[1], apInfo.bssid[2],
                 apInfo.bssid[3], apInfo.bssid[4], apInfo.bssid[5]);
    }
  }
  // v3.9.38: distinguish the first WiFi association after boot from a true
  // WiFi recovery.  v3.9.35-3.9.37 called Blynk.disconnect() and tore down the
  // raw transport even on the very first association. That was intended to
  // clean stale sessions after a WiFi failure, but it also broke the clean
  // power-on path on this firmware/Blynk 0.6.1 combination. The first boot
  // must use the normal Blynk.config() path; only a real WiFi re-association
  // needs the aggressive stale-session teardown.
  const bool firstWifiAssociation = (wifiSuccessfulConnections == 0);

  // Clean power-on uses the normal Blynk startup path. Forced recovery is
  // enabled only for a later WiFi re-association.
  if (firstWifiAssociation) blynkForceReconnect = false;

  wifiConnectInProgress = false;
  wifiFastReauthAttempts = 0;
  wifiFastReauthNextMillis = 0;
  wifiFastReauthScheduled = false;
  wifiDisconnectEventPending = false;
  wifiExpectedDisconnectUntilMs = 0;
  wifiOutageInitialDisconnectReason = 0;
  wifiOutageInitialDisconnectEventMs = 0;
  wifiSuccessfulConnections++;
  wifiConsecutiveFailures = 0;
  wifiOfflineSince = 0;
  wifiHardRebuildUsedThisOutage = false;
  sharedPathForceFullScan = false;
  wifiLastGotIpMillis = millis();
  DBG_PRINTF("\n[WiFi] Connected — IP=%s RSSI=%ddBm channel=%d BSSID=%s MAC=%s disconnectReason=%u\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel(),
                WiFi.BSSIDstr().c_str(), WiFi.macAddress().c_str(),
                (unsigned)wifiLastDisconnectReason);
  // v3.9.38: only force a Blynk/session teardown when this is a REAL WiFi
  // recovery. On a clean power boot there is no stale Blynk session to tear down,
  // and the v3.9.35-3.9.37 teardown could leave the Blynk protocol state in a
  // condition where the first bounded probe never started.
  if (!firstWifiAssociation) {
    // A real WiFi reassociation creates a new receive path. Discard the old
    // socket generation and its timeout baselines; a driver rebuild remains
    // limited to one attempt until a fully authenticated Blynk session returns.
    blynkSequenceHoleRecoveryTrial = false;
    blynkSequenceHoleApplicationCanary = false;
    blynkApplicationCanaryPassed = 0;
    blynkApplicationCanarySentBaseline = 0;
    blynkApplicationCanaryVerifiedBaseline = 0;
    blynkSequenceHoleRecoveryStartedMillis = 0;
    blynkSequenceHoleSocketTrialCount = 0;
    blynkSequenceHoleRecentGenerations = 0;
    blynkSequenceHoleLastGenerationMillis = 0;
    blynkStockOutageTcpConnectFailures = 0;
    blynkStockOutageConnectFailureBaseline = blynkTransportConnectFailures;
    blynkStockOutageTcpConnectTimeouts = 0;
    blynkStockOutageConnectTimeoutBaseline = blynkTransportConnectTimeouts;
    blynkStockWifiStackRecoveryInProgress = false;
    // v3.9.39: force the next Blynk connection attempt to start even if the
    // Blynk protocol object still reports connected. This is the exact failure
    // seen after a WiFi loss/re-association: WiFi comes back, but the stale
    // Blynk session can otherwise block the probe gate.
    blynkForceReconnect = true;

    // WiFi can reconnect while the Blynk protocol object still believes it is
    // connected. Blynk.config() only changes the configured server; it does NOT
    // force the old protocol/session state to disconnected. Tear down the old
    // session and raw socket before rebuilding the Blynk connection.
#if BLYNK_FORENSIC_LOCK
    DBG_PRINTLN("[Blynk FORENSIC] WiFi re-association: NOT calling Blynk.disconnect()");
#else
    Blynk.disconnect();
#if CONNECTION_MODE == 0
    _stockBlynkTransportDiagnostics.disconnect();
#endif
#endif
  }

  blynkProbeAbort();
  blynkProbeInFlight = false;
  blynkConfigured = false;
  blynkHandshakeActive = false;
  blynkHandshakeStarted = 0;
  blynkOfflineSince = 0;
  blynkLastConnectedMillis = 0;
  blynkLastRxMillis = millis();
  // v3.9.212: a clean boot waits briefly after its first GOT_IP. Field traces
  // repeatedly show that the immediate attempt consumes its full one-second
  // TCP budget, while the following attempt succeeds. A true re-association
  // remains immediate because outage recovery should not incur boot grace.
  const uint32_t blynkEligibleAt = millis() +
      (firstWifiAssociation ? BLYNK_FIRST_ASSOCIATION_GRACE_MS : 0UL);
  blynkConnectImmediate = !firstWifiAssociation;
  lastBlynkConnectAttempt = millis();
  blynkStockNextRunMillis = blynkEligibleAt;

  // Blynk.config() sets server address only — does NOT block or spin.
  // STOCK BlynkSimpleEsp32 establishes the session from Blynk.run(); the loop
  // gates disconnected calls so the stock WiFiClient cannot retry continuously.
  Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
  blynkConfigured = true;
#if CONNECTION_MODE == 1
  // Legacy BlynkProtocol::disconnect() sets state=DISCONNECTED, while
  // config()/begin() only changes credentials and endpoint. Explicitly put
  // the protocol back into CONNECTING without entering its blocking loop.
  (void)Blynk.connect(0);
#endif
  blynkHandshakeActive = false;
  blynkHandshakeStarted = 0;
  blynkStartupHoldoffUntil = blynkEligibleAt;
  if (firstWifiAssociation) {
    Serial.printf("[Blynk STARTUP GRACE] first GOT_IP; delaying initial socket %lums\n",
                  (unsigned long)BLYNK_FIRST_ASSOCIATION_GRACE_MS);
  }
  if (firstWifiAssociation) {
    engMsgf("WiFi OK — Blynk config set %s:%d (initial handshake pending)", BLYNK_SERVER, BLYNK_PORT);
  } else {
    engMsgf("WiFi OK — Blynk transport reset; config set %s:%d (handshake pending)", BLYNK_SERVER, BLYNK_PORT);
  }
  buzzerTone(5000, 100);
}

void wifiService()
{
  if (!WIFI) return;

  const uint32_t serviceNow = millis();
  if (blynkSharedPathRecoveryService(serviceNow)) return;

  const bool freshGotIpAfterDisconnect = wifiLastGotIpEventMs != 0 &&
      (int32_t)(wifiLastGotIpEventMs - wifiLastDisconnectEventMs) > 0;
  const bool unresolvedUnexpectedDisconnect =
      wifiDisconnectEventPending && !wifiLastDisconnectWasExpected &&
      !freshGotIpAfterDisconnect;
  const bool observationStillAwaitingGotIp = wifiUnexpectedObservationActive &&
      !(wifiLastGotIpEventMs != 0 &&
        (int32_t)(wifiLastGotIpEventMs - wifiUnexpectedObservationStartedMs) > 0);

  if (WiFi.status() == WL_CONNECTED &&
      !unresolvedUnexpectedDisconnect && !observationStillAwaitingGotIp) {
    // Keep exactly one WiFi reconnect owner.  A real disconnect event is
    // handled below by wifiService(); do not arm Arduino's parallel reconnect
    // state machine while the custom state machine is also active.
    WiFi.setAutoReconnect(false);
    // v3.9.63: a spontaneous disconnect is allowed to recover naturally for
    // WIFI_UNEXPECTED_OBSERVATION_MS before we touch the WiFi driver.  If the
    // station came back by itself, record that as a natural recovery and do
    // NOT perform our disconnect/reconnect teardown.
    if (wifiUnexpectedObservationActive) {
      const uint32_t obsAge = millis() - wifiUnexpectedObservationStartedMs;
      wifiUnexpectedObservationActive = false;
      wifiUnexpectedObservationUntilMs = 0;
      wifiDisconnectEventPending = false;
      DBG_PRINTF("[WiFi OBSERVE] NATURAL RECOVERY — STA_CONNECTED=%lu GOT_IP=%lu obsAge=%lums reason=%u unexpectedDisconnects=%lu IP=%s GW=%s RSSI=%d channel=%d BSSID=%s\n",
                 (unsigned long)wifiStaConnectedEventCount,
                 (unsigned long)wifiGotIpEventCount,
                 (unsigned long)obsAge,
                 (unsigned)wifiLastDisconnectReason,
                 (unsigned long)wifiUnexpectedDisconnectCount,
                 WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(),
                 WiFi.RSSI(), WiFi.channel(), WiFi.BSSIDstr().c_str());
      wifiMarkConnected();
    } else if (wifiConnectInProgress ||
               (wifiDisconnectEventPending && freshGotIpAfterDisconnect)) {
      wifiDisconnectEventPending = false;
      wifiMarkConnected();
      DBG_PRINTF("[WiFi EVENT] GOT_IP observed — eventCount=%lu staConnected=%lu gotIpEvents=%lu lastEventAge=%lums lastStaConnectedAge=%lums IP=%s GW=%s RSSI=%d channel=%d BSSID=%s\n",
                 (unsigned long)wifiEventCount,
                 (unsigned long)wifiStaConnectedEventCount,
                 (unsigned long)wifiGotIpEventCount,
                 (unsigned long)(wifiLastGotIpEventMs ? millis() - wifiLastGotIpEventMs : 0),
                 (unsigned long)(wifiLastStaConnectedEventMs ? millis() - wifiLastStaConnectedEventMs : 0),
                 WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(),
                 WiFi.RSSI(), WiFi.channel(), WiFi.BSSIDstr().c_str());
    }
    return;
  }


  unsigned long now = millis();
  if (wifiConnectInProgress) {
    // Do not block Core 1 while the WiFi driver performs its association.
    // If it takes too long, abandon this attempt and allow a later retry.
    if (now - wifiConnectStarted >= WIFI_ASSOC_ATTEMPT_TIMEOUT_MS) {
      wifiConnectInProgress = false;
      if (wifiConsecutiveFailures < 0xFFFFFFFFUL) ++wifiConsecutiveFailures;
      if (wifiOfflineSince == 0) wifiOfflineSince = now;
      const uint32_t failedAttemptCount = wifiConsecutiveFailures;
      const uint32_t offlineAgeMs = now - wifiOfflineSince;

      // The shared-path state already performed the only authorized radio/
      // netif rebuild, cleared the old STA config and forced a full scan. If
      // that clean attempt still cannot obtain GOT_IP, more disconnect/begin
      // cycles merely repeat the v3.9.204 two-minute outage. A controlled MCU
      // restart is the next distinct and field-proven reset domain.
      if (sharedPathRecoveryState == SHARED_PATH_WAIT_GOT_IP) {
        Serial.printf("[Blynk SHARED PATH RECOVERY] clean full-scan association timed out after %lums reason=%u; no second WiFi teardown, restarting MCU\n",
                      (unsigned long)(now - sharedPathRecoveryStateAtMs),
                      (unsigned)wifiLastDisconnectReason);
        diagnosticResetRequest("SHARED_PATH_CLEAN_ASSOC_TIMEOUT");
        return;
      }

      const bool useOneHardRebuild =
          !wifiHardRebuildUsedThisOutage &&
          failedAttemptCount >= WIFI_HARD_REBUILD_AFTER_FAILURES;
      // Association startup is independent of Blynk recovery. On timeout,
      // abandon only this incomplete attempt and retry after a short settle;
      // never infer from a Blynk fault that the radio must be power-cycled.
      // A 20-second attempt is no longer useful driver state. WiFi.reconnect()
      // maps directly to esp_wifi_connect() and can collide with the still-
      // active attempt, exactly as seen in the field trace. Cancel/reset once,
      // wait for STA shutdown, then let wifiStartConnect() create one new scan.
      DBG_PRINTF("[WiFi] CONNECT_TIMEOUT - staged single-owner recovery failure=%lu hardAt=%u offline=%lums hardUsed=%d hardNow=%d initialReason=%u latestReason=%u\n",
                 (unsigned long)failedAttemptCount,
                 (unsigned)WIFI_HARD_REBUILD_AFTER_FAILURES,
                 (unsigned long)offlineAgeMs,
                 wifiHardRebuildUsedThisOutage ? 1 : 0,
                 useOneHardRebuild ? 1 : 0,
                 (unsigned)wifiOutageInitialDisconnectReason,
                 (unsigned)wifiLastDisconnectReason);
      wifiIntentionalTeardown(useOneHardRebuild ?
          "WIFI_ONE_HARD_REBUILD" :
          "WIFI_ASSOC_TIMEOUT_SOFT_CANCEL", useOneHardRebuild);
      if (useOneHardRebuild) wifiHardRebuildUsedThisOutage = true;
      wifiDisconnectSettleUntil = now + WIFI_DISCONNECT_SETTLE_MS;
      wifiReconnectPending = true;
      DBG_PRINTF("\n[WiFi] FAILED (%lums timeout) consecutive=%lu reason=%u eventAge=%lums\n",
                 (unsigned long)WIFI_ASSOC_ATTEMPT_TIMEOUT_MS,
                  (unsigned long)failedAttemptCount,
                 (unsigned)wifiLastDisconnectReason,
                 (unsigned long)(now - wifiLastDisconnectEventMs));
      DBG_PRINTF("[WiFi DIAG] status=%d RSSI=%d channel=%d BSSID=%s IP=%s GW=%s\n",
                 (int)WiFi.status(), WiFi.RSSI(), WiFi.channel(),
                 WiFi.BSSIDstr().c_str(), WiFi.localIP().toString().c_str(),
                 WiFi.gatewayIP().toString().c_str());
      DBG_PRINTF("[WiFi DIAG] attempts=%lu successes=%lu consecutiveFailures=%lu offlineAge=%lums\n",
                  (unsigned long)wifiConnectAttempts,
                  (unsigned long)wifiSuccessfulConnections,
                  (unsigned long)failedAttemptCount,
                  (unsigned long)offlineAgeMs);
      DBG_PRINTF("[WiFi EVENT DIAG] eventCount=%lu staConnected=%lu gotIpEvents=%lu lastEventId=%lu lastEventAge=%lums lastDisconnectReason=%u expected=%d unexpectedDisconnects=%lu lastTeardown=%s teardownAge=%lums teardownCount=%lu\n",
                 (unsigned long)wifiEventCount,
                 (unsigned long)wifiStaConnectedEventCount,
                 (unsigned long)wifiGotIpEventCount,
                 (unsigned long)wifiLastEventId,
                 (unsigned long)(wifiLastEventMs ? now - wifiLastEventMs : 0),
                 (unsigned)wifiLastDisconnectReason,
                 wifiLastDisconnectWasExpected ? 1 : 0,
                 (unsigned long)wifiUnexpectedDisconnectCount,
                 wifiLastTeardownReason,
                 (unsigned long)(wifiLastTeardownMillis ? now - wifiLastTeardownMillis : 0),
                 (unsigned long)wifiIntentionalTeardownCount);
      DBG_PRINTF("[FAULT SNAPSHOT] reset=%s stage=%s core1Max=%lums core1Stalls=%lu heap=%u minHeap=%u\n",
                 bootResetReasonText, core1StageText(core1StageId),
                 (unsigned long)core1MaxStageMs, (unsigned long)core1StallCount,
                 ESP.getFreeHeap(), ESP.getMinFreeHeap());
      DBG_PRINTF("[FAULT SNAPSHOT] I2C SDA=%d SCL=%d I2Cerrors=%lu probesFailed=%lu recoveries=%lu COphase=%u duty=%u heaterV=%.2f\n",
                 core0SensorDiag.i2cSdaLevel, core0SensorDiag.i2cSclLevel,
                 (unsigned long)core0SensorDiag.i2cErrorCount,
                 (unsigned long)core0SensorDiag.i2cProbeFailCount,
                 (unsigned long)core0SensorDiag.i2cRecoveryCount,
                 (unsigned)core0SensorDiag.coPhase, (unsigned)core0SensorDiag.coDuty,
                 (double)core0SensorDiag.coHeaterV);
      engMsg("WiFi connect attempt timed out - retrying");
      // One owner only: wifiService() schedules the retry after the driver
      // settle. Do not enable auto-reconnect here; doing so starts a second
      // A field reset restores this exact persistent AUTH_EXPIRE condition.
      // Keep that as a bounded last resort, only after soft retries and the
      // single hard rebuild have both failed for four continuous minutes.
      if (wifiHardRebuildUsedThisOutage &&
          offlineAgeMs >= WIFI_FINAL_RECOVERY_RESTART_MS) {
        Serial.printf("[WiFi FINAL RECOVERY] offline=%lums failures=%lu after one hard rebuild; restarting MCU once\n",
                      (unsigned long)offlineAgeMs,
                      (unsigned long)failedAttemptCount);
        delay(100);
        ESP.restart();
      }
      // Preserve the original audible failure indication. It is emitted only
      // once per failed connection attempt; the connection process itself is
      // still non-blocking for the full 20-second association window.
      wifiFailureAlertAt = now;
      buzzerPattern3(1000, 600, 200);
    }
    return;
  }

  // v3.9.14 FIX: never call wifiStartConnect() (WiFi.begin()) in the same
  // pass as a WiFi.disconnect() — see wifiDisconnectSettleUntil above. Wait
  // out the short settle window first; this check is non-blocking, it simply
  // defers the retry by at most WIFI_DISCONNECT_SETTLE_MS. Once settled, fire
  // the reconnect immediately if one was requested (wifiReconnectPending),
  // rather than waiting on the unrelated periodic lastWiFiCheck gate below.
  if (wifiDisconnectSettleUntil != 0) {
    if (now < wifiDisconnectSettleUntil) return;
    wifiDisconnectSettleUntil = 0;
    if (wifiReconnectPending) {
      wifiReconnectPending = false;
      wifiStartConnect();
      return;
    }
  }

  // v3.9.63: unexpected disconnects are diagnostic opportunities first, not
  // immediate recovery triggers.  Leave the ESP32 WiFi driver alone for 30s
  // so we can observe whether the AP/driver naturally reassociates.
  if (wifiDisconnectEventPending && !wifiLastDisconnectWasExpected &&
      !wifiUnexpectedObservationActive) {
    wifiDisconnectEventPending = false;
    wifiUnexpectedObservationActive = true;
    wifiUnexpectedObservationStartedMs = now;
    wifiUnexpectedObservationUntilMs = now + WIFI_UNEXPECTED_OBSERVATION_MS;
    wifiUnexpectedObservationLastLogMs = 0;
    wifiFastReauthAttempts = 0;
    // Arduino-ESP32 3.3.12 STA.cpp has a process-lifetime `first_connect`
    // fallback: its first disconnect always calls disconnect()+connect(), even
    // when setAutoReconnect(false). The field trace proved that our additional
    // +500ms call still collided with it (ESP_ERR_WIFI_CONN / 0x3007). Let that
    // mandatory core retry exclusively own event #1. For later outage
    // generations, first_connect is already false and our manual retry owns the
    // event as before.
    const bool arduinoCoreOwnsFirstRetry = (wifiDisconnectEventSequence == 1U);
    wifiFastReauthNextMillis = arduinoCoreOwnsFirstRetry ? 0 : now + 500UL;
    wifiFastReauthScheduled = !arduinoCoreOwnsFirstRetry;
    DBG_PRINTF("[WiFi OBSERVE] Unexpected STA_DISCONNECTED — reason=%u. Holding recovery for %lums to allow natural STA reconnect. seq=%lu unexpected=%lu RSSI=%d channel=%d BSSID=%s\n",
               (unsigned)wifiLastDisconnectReason,
               (unsigned long)WIFI_UNEXPECTED_OBSERVATION_MS,
               (unsigned long)wifiDisconnectEventSequence,
               (unsigned long)wifiUnexpectedDisconnectCount,
               WiFi.RSSI(), WiFi.channel(), WiFi.BSSIDstr().c_str());
    Serial.printf("[WiFi FAST REAUTH] reason=%u owner=%s scheduledIn=%lums observation=%lums\n",
                  (unsigned)wifiLastDisconnectReason,
                  arduinoCoreOwnsFirstRetry ? "ARDUINO_CORE_FIRST_RETRY" : "SKETCH",
                  (unsigned long)(arduinoCoreOwnsFirstRetry ? 0UL : 500UL),
                  (unsigned long)WIFI_UNEXPECTED_OBSERVATION_MS);

    // v3.9.133 DIAGNOSTIC: reason=204 (WIFI_REASON_HANDSHAKE_TIMEOUT) is a
    // WPA2 4-way EAPOL handshake timing failure, not a signal-strength
    // issue — RSSI can be excellent and this still fires. Since other
    // devices on the same AP (including an ESP8266 on the same Blynk
    // server) do not show this, the AP/router is not the primary suspect;
    // what differs is this device's own workload: dual-core FreeRTOS with
    // I2S mic, GPS UART + PPS ISR, I2C sensors, and LEDC PWM all running
    // concurrently with WiFi. A handshake timeout is a TIMING failure —
    // if any of this device's own ISRs or tasks were starving the WiFi
    // driver's task of CPU at the moment the AP initiated a re-key/
    // handshake, that would produce exactly this reason code on a
    // perfectly healthy RF link. core1StageId/core1MaxStageMs/
    // core1StallCount capture what Core 1 (WiFi's own core) was doing;
    // I2Cerrors is logged to check for correlation with I2C bus activity.
    Serial.printf("[WiFi HANDSHAKE CONTEXT] reason=%u core1Stage=%s core1StageMaxMs=%lu Core1Stalls=%lu heap=%u minHeap=%u I2Cerrors=%lu\n",
                  (unsigned)wifiLastDisconnectReason,
                  core1StageText(core1StageId),
                  (unsigned long)core1MaxStageMs,
                  (unsigned long)core1StallCount,
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                  (unsigned long)core0SensorDiag.i2cErrorCount);
    return;
  }

  if (wifiUnexpectedObservationActive) {
    if ((int32_t)(wifiUnexpectedObservationUntilMs - now) > 0) {
      // Every failed association raises another STA_DISCONNECTED event. Retry
      // that event directly instead of ignoring it until a long timeout. No
      // disconnect(), mode-off or Blynk teardown occurs inside this window.
      if (wifiDisconnectEventPending && !wifiLastDisconnectWasExpected) {
        wifiDisconnectEventPending = false;
        wifiFastReauthScheduled = true;
        wifiFastReauthNextMillis = now + 500UL;
      }
      if (wifiFastReauthScheduled &&
          wifiFastReauthAttempts < WIFI_FAST_REAUTH_MAX_ATTEMPTS &&
          (int32_t)(now - wifiFastReauthNextMillis) >= 0) {
        wifiFastReauthScheduled = false;
        const esp_err_t retryResult = esp_wifi_connect();
        ++wifiFastReauthAttempts;
        Serial.printf("[WiFi FAST REAUTH] attempt=%u/%u result=%d initialReason=%u latestReason=%u; awaiting GOT_IP or next disconnect event\n",
                      (unsigned)wifiFastReauthAttempts,
                      (unsigned)WIFI_FAST_REAUTH_MAX_ATTEMPTS,
                      (int)retryResult,
                      (unsigned)wifiOutageInitialDisconnectReason,
                      (unsigned)wifiLastDisconnectReason);
      }
      if (wifiUnexpectedObservationLastLogMs == 0 ||
          now - wifiUnexpectedObservationLastLogMs >= 5000UL) {
        wifiUnexpectedObservationLastLogMs = now;
        Serial.printf("[WiFi OBSERVE] age=%lums remaining=%lums reason=%u status=%d Blynk=%d RSSI=%d channel=%d BSSID=%s IP=%s GW=%s staConnected=%lu gotIP=%lu blynkProxyRxAge=%lums runBlockMax=%lums\n",
                      (unsigned long)(now - wifiUnexpectedObservationStartedMs),
                      (unsigned long)(wifiUnexpectedObservationUntilMs - now),
                      (unsigned)wifiLastDisconnectReason, (int)WiFi.status(),
                      Blynk.connected() ? 1 : 0, WiFi.RSSI(), WiFi.channel(),
                      WiFi.BSSIDstr().c_str(), WiFi.localIP().toString().c_str(),
                      WiFi.gatewayIP().toString().c_str(),
                      (unsigned long)wifiStaConnectedEventCount,
                      (unsigned long)wifiGotIpEventCount,
                      blynkStockRxProxyAgeMs(now),
                      (unsigned long)blynkStockRunBlockMaxMs);
      }
      return;
    }

    wifiUnexpectedObservationActive = false;
    wifiUnexpectedObservationUntilMs = 0;
    wifiFastReauthScheduled = false;
    wifiDisconnectEventPending = false;
    DBG_PRINTF("[WiFi OBSERVE] Fast recovery did not complete within %lums; cancelling the old attempt and rebuilding STA before a full scan. initialReason=%u latestReason=%u attempts=%u staConnected=%lu gotIP=%lu\n",
               (unsigned long)WIFI_UNEXPECTED_OBSERVATION_MS,
               (unsigned)wifiOutageInitialDisconnectReason,
               (unsigned)wifiLastDisconnectReason,
               (unsigned)wifiFastReauthAttempts,
               (unsigned long)wifiStaConnectedEventCount,
               (unsigned long)wifiGotIpEventCount);
    wifiIntentionalTeardown("FAST_REAUTH_TIMEOUT_DRIVER_REBUILD", true);
    wifiDisconnectSettleUntil = now + WIFI_DISCONNECT_SETTLE_MS;
    wifiReconnectPending = true;
    return;
  }

  if (now - lastWiFiCheck >= WIFI_RECONNECT_MS || wifiDisconnectEventPending) {
    // v3.9.41: react immediately to a real disconnect event instead of only
    // a 30s poll. wifiDisconnectEventPending is set from the WiFi event
    // callback the instant ARDUINO_EVENT_WIFI_STA_DISCONNECTED fires; the
    // WIFI_RECONNECT_MS timer remains only as a safety net in case an event
    // is ever missed. Log the reason/RSSI right here, at the moment we react,
    // since that is the only place that can tell us *why* — the 10s summary
    // elsewhere only shows the last known values by the time it prints.
    const bool eventTriggered = wifiDisconnectEventPending;
    wifiDisconnectEventPending = false;
    lastWiFiCheck = now;
    DBG_PRINTF("[WiFi] Reconnect triggered by %s — reason=%u expected=%d eventAge=%lums lastRSSI=%d seq=%lu unexpected=%lu\n",
               eventTriggered ? "DISCONNECT EVENT" : "30s poll (no event seen)",
               (unsigned)wifiLastDisconnectReason,
               wifiLastDisconnectWasExpected ? 1 : 0,
               (unsigned long)(now - wifiLastDisconnectEventMs),
               WiFi.RSSI(),
               (unsigned long)wifiDisconnectEventSequence,
               (unsigned long)wifiUnexpectedDisconnectCount);
    DBG_PRINTF("[WiFi EVENT] lastEventId=%lu lastEventAge=%lums disconnectReason=%u expected=%d status=%d RSSI=%d channel=%d BSSID=%s IP=%s GW=%s\n",
               (unsigned long)wifiLastEventId,
               (unsigned long)(wifiLastEventMs ? now - wifiLastEventMs : 0),
               (unsigned)wifiLastEventReason,
               wifiLastDisconnectWasExpected ? 1 : 0,
               (int)WiFi.status(), WiFi.RSSI(), WiFi.channel(),
               WiFi.BSSIDstr().c_str(), WiFi.localIP().toString().c_str(),
               WiFi.gatewayIP().toString().c_str());
    engMsgf("WiFi: reconnect (reason=%u expected=%d)",
            (unsigned)wifiLastDisconnectReason,
            wifiLastDisconnectWasExpected ? 1 : 0);
#if BLYNK_FORENSIC_LOCK
    DBG_PRINTLN("[Blynk FORENSIC] WiFi recovery: NOT calling Blynk.disconnect()");
#else
    Blynk.disconnect();
#if CONNECTION_MODE == 0
    _stockBlynkTransportDiagnostics.disconnect();
#endif
#endif
    // WiFi is already disconnected. Start association without first issuing
    // disconnect()/WIFI_OFF; only the bounded failure-limit branch may rebuild
    // the radio. wifiStartConnect() uses the cached AP first and later falls
    // back to an unconstrained scan.
    wifiStartConnect();
  }
}

// ============================================================================
// WiFi CONNECT
// ============================================================================

// Kept as the original blocking helper for compatibility with the existing
// firmware interface. Normal operation now uses wifiStartConnect()/wifiService()
// so Core 1 is never held in a 20-second connection loop.
bool wifiConnect()
{
  DBG_PRINTF("[WiFi] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t<20000) { delay(500); Serial.print("."); }
  if (WiFi.status()!=WL_CONNECTED) {
    DBG_PRINTLN("\n[WiFi] FAILED");
    engMsg("WiFi FAILED: offline");
    buzzerPattern3(1000, 600, 200);
    return false;
  }
  DBG_PRINTF("\n[WiFi] Connected — IP=%s RSSI=%ddBm MAC=%s\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.macAddress().c_str());
  // Blynk.config() sets server address only — does NOT block or spin.
  // Blynk.run() in loop() completes the handshake on first call.
  // DO NOT use Blynk.connect(timeout) — it spins internally for 5s and causes IWDT crash.
  Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
  engMsgf("WiFi OK — Blynk config set %s:%d (connected=%d)", BLYNK_SERVER, BLYNK_PORT, Blynk.connected() ? 1 : 0);
  buzzerTone(5000, 100);
  return true;
}

// ============================================================================
// SETUP — runs on Core 1 (Arduino default app_cpu)
// ============================================================================

// ─── V55 REMOTE HARD RESET ────────────────────────────────────────────────────
// Configure V55 in Blynk as a Button widget (Push is recommended).
// A HIGH/1 write requests a firmware-triggered ESP32 restart.
//
// The RTC checkpoint is written independently and survives this software
// restart, so V51 can report the previous run after the device comes back.
// v3.9.2 FIX: the V55 reset is performed directly from the Blynk callback.
// esp_restart() is allowed to run with interrupts enabled; noInterrupts() is
// deliberately NOT used because it can prevent the ESP32 restart machinery
// from completing and trigger the Interrupt-WDT.
// The short delay after Serial.flush() only gives the reset message time to
// leave the UART before esp_restart() starts the system restart.
//
// The RTC checkpoint is written independently and survives this software
// restart, so V51 can report the previous run after the device comes back.

// V55 — Blynk hard-reset button.
// Configure V55 in Blynk as a Button widget; send 1 when pressed.
//
// IMPORTANT: Arduino's sketch preprocessor generates prototypes for functions
// defined in .ino files. With Blynk 0.6.1, BlynkWidgetWrite55 is declared by
// BlynkHandlers.h with C linkage, so an auto-generated C++ prototype conflicts
// with the library declaration. The actual handler is therefore kept in a
// separate .h file. Arduino does not prototype-generate functions in .h files.
//
// Keep this include AFTER the Blynk headers have been included above.
#include "V55Handler.h"


// v3.9.107: watchdog forensic heartbeat.  This survives TG0/TG1 watchdog
// resets in RTC memory and records the most recent Core-1 activity every
// 100ms.  It does not write flash and does not disable any watchdog.
#define WDT_FORENSIC_MAGIC 0x57445446UL
#define WDT_FORENSIC_VERSION 1U
#define WDT_FORENSIC_INTERVAL_MS 100UL
struct WdtForensicRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t stageId;
  uint32_t sequence;
  uint32_t millisAtUpdate;
  uint32_t loopCount;
  uint8_t wifiConnected;
  uint8_t blynkConnected;
  uint8_t v61Value;
  uint8_t reserved;
  uint32_t v61Sequence;
  uint32_t lastTxElapsedMs;
  uint32_t lastBlynkRunMs;
  uint32_t lastCore1ProgressMs;
};
RTC_NOINIT_ATTR WdtForensicRecord wdtForensicRecord = {};
static uint32_t wdtForensicLastWriteMs = 0;
static uint32_t wdtForensicLoopCount = 0;

static void wdtForensicHeartbeat()
{
  const uint32_t now = millis();
  if (wdtForensicLastWriteMs != 0 &&
      (uint32_t)(now - wdtForensicLastWriteMs) < WDT_FORENSIC_INTERVAL_MS) return;
  wdtForensicLastWriteMs = now;
  WdtForensicRecord &r = wdtForensicRecord;
  r.magic = WDT_FORENSIC_MAGIC;
  r.version = WDT_FORENSIC_VERSION;
  r.stageId = core1StageId;
  r.sequence++;
  r.millisAtUpdate = now;
  r.loopCount = wdtForensicLoopCount;
  r.wifiConnected = (WiFi.status() == WL_CONNECTED) ? 1U : 0U;
  r.blynkConnected = Blynk.connected() ? 1U : 0U;
  r.v61Value = (v61TestValue == 0U) ? 9U : v61TestValue;
  r.v61Sequence = v61TestSequence;
  r.lastTxElapsedMs = blynkTxLastCallMs;
  r.lastBlynkRunMs = perfMaxBlynkRunMs;
  const uint32_t progressUs = core1LastProgressUs;
  r.lastCore1ProgressMs = (uint32_t)((micros() - progressUs) / 1000UL);
}

static void wdtForensicReportPrevious()
{
  const WdtForensicRecord &r = wdtForensicRecord;
  if (r.magic != WDT_FORENSIC_MAGIC || r.version != WDT_FORENSIC_VERSION) {
    Serial.println("[WDT FORENSIC] no valid previous RTC-NOINIT heartbeat");
    return;
  }
  Serial.printf("[WDT FORENSIC] previous seq=%lu at=%lums loop=%lu stage=%u(%s) wifi=%u blynk=%u V61seq=%lu nextV61=%u lastTx=%lums blynkRunMax=%lums core1ProgressAge=%lums\n",
                (unsigned long)r.sequence,
                (unsigned long)r.millisAtUpdate,
                (unsigned long)r.loopCount,
                (unsigned)r.stageId,
                core1StageText(r.stageId),
                (unsigned)r.wifiConnected,
                (unsigned)r.blynkConnected,
                (unsigned long)r.v61Sequence,
                (unsigned)r.v61Value,
                (unsigned long)r.lastTxElapsedMs,
                (unsigned long)r.lastBlynkRunMs,
                (unsigned long)r.lastCore1ProgressMs);
}

static void core0StallRecoveryReportPrevious()
{
  const Core0StallRecoveryRecord r = core0StallRecoveryRecord;
  if (r.magic != CORE0_STALL_RTC_MAGIC) return;
  Serial.printf("[CORE0 HARD STALL RECOVERY] count=%lu detectedAt=%lums age=%lums stage=%u(%s)\n",
                (unsigned long)r.resetCount,
                (unsigned long)r.detectedAtMs,
                (unsigned long)r.progressAgeMs,
                (unsigned)r.stageId,
                core1StageText(r.stageId));
  core0StallRecoveryRecord.magic = 0;
}

void setup()
{
  // v3.9.125 FIX: field logs showed the CONNECTING-state Blynk.run() call
  // (conn.connect() + sendCmd(HW_LOGIN)) taking a highly repeatable
  // ~2.8-3.0s, with every socket-level primitive (connect/write/select)
  // independently timed and cleared as the cause across several rounds of
  // instrumentation. The one thing never checked: this call path also
  // emits several Serial.printf()/DBG_PRINTF() lines (the "Connecting
  // to..." message, BLYNK_DBG_DUMP's hex/ASCII dump of the outgoing
  // HW_LOGIN packet, connect success diagnostics) — all real execution
  // time from inside Blynk.run(), not scheduler preemption (confirmed by
  // the earlier yieldGapMs=0 split). Serial.printf() blocks once the TX
  // ring buffer fills if nothing drains it fast enough (a disconnected/
  // slow USB-CDC host, or simply a burst larger than the default buffer),
  // and that block is invisible to every socket/select timer since it
  // isn't socket I/O at all. Sizing the TX buffer well above the largest
  // single burst this path can produce removes that as a possibility;
  // if the stall persists after this, it conclusively rules out Serial
  // entirely.
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  Serial.printf("[INIT] Blynk connection mode: stock Blynk 0.6.1 protocol / nonblocking deferred-TX WiFiClient\n");
  Serial.printf("[INIT] Blynk timeout: %lums, heartbeat: %us, forced-refresh: OFF\n",
                (unsigned long)BLYNK_TIMEOUT_MS, (unsigned)BLYNK_HEARTBEAT);
  Serial.printf("[INIT] Blynk heartbeat=%us; library timeout=%lums; heartbeat deadline=%lums\n",
                (unsigned)BLYNK_HEARTBEAT,
                (unsigned long)BLYNK_TIMEOUT_MS,
                (unsigned long)(BLYNK_HEARTBEAT * 1000UL + 3UL * BLYNK_TIMEOUT_MS));
  // v3.9.134 CONFIRMED (field evidence, not inference): the per-ping RTT
  // watch added in v3.9.132 caught a real, measured miss — msgId=61441
  // UNANSWERED after 132867ms, with answeredCount=38 vs unansweredCount=7
  // in one run (~15% loss on this specific device-ping -> server-response
  // exchange). WiFi was solid throughout (wifiDiscEvents=0, RSSI -54 to
  // -60dBm) and V61 TX never failed, so this is not a WiFi, transport, or
  // firmware-logic problem — the device does everything correctly and
  // correctly detects the miss. The loss is happening between this
  // device and the server (server-side handling or the network path
  // between them), not on the device. Next step to localize it further is
  // a tcpdump on the RPi5 filtered to this device's IP during a future
  // occurrence, to see whether the PING packet reaches the server's NIC
  // and whether the server ever sends a reply out.
  //
  // Also confirmed resolved this session: the TG1WDT_SYS_RESET crash-loop
  // with repeating "Fatal exception (29): StoreProhibited" at
  // epc1=0x4000c28b (ROM address, caller unresolvable without a full
  // backtrace) did not recur after Erase All Flash + a clean recompile
  // (stale Arduino sketch build-cache folder deleted). Likely a corrupted
  // flash image from a prior interrupted write, not an application bug.
  Serial.printf("\n[INIT] Air Quality Station ESP32 v%s — Arduino core %s — GPS 115200 baud + 10Hz config\n",
                FIRMWARE_VERSION, ESP_ARDUINO_VERSION_STR);
  Serial.printf("[INIT] ten_mins_autoreset = %s\n", ten_mins_autoreset ? "ON":"OFF");
  Serial.printf("[INIT] DEBUGON = %s\n", DEBUGON ? "ON" : "OFF");

  // ── ESP reset-cause diagnostic ───────────────────────────────────────────
  bootResetReason = esp_reset_reason();
  bootResetReasonText = resetReasonToString(bootResetReason);
  Serial.printf("[INIT] ESP reset reason: %s (%d)\n",
                bootResetReasonText, (int)bootResetReason);
  wdtForensicReportPrevious();
  core0StallRecoveryReportPrevious();
  // v3.9.81: TG0WDT_SYS_RESET is a watchdog-class reset. A subsequent
  // bootloader "csum err" is a separate flash-image checksum/read problem;
  // it is not caused by esp_restart() itself and cannot be repaired by code
  // running after boot. Keep the two events distinct in diagnostics.

  // Read the previous RTC-SRAM checkpoint before this run starts writing it.
  rtcReportPreviousCheckpoint();

  // ── Explicit ESP task-watchdog configuration ─────────────────────────────
  // Arduino-ESP32 3.x / ESP-IDF 5.x changed esp_task_wdt_init() to accept
  // esp_task_wdt_config_t. Preserve the previous 10-second + panic intent.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 20000,
    // Monitor only tasks explicitly registered with esp_task_wdt_add().
    // Idle-task scheduling is an OS load signal, not proof that this
    // application is wedged, and produced avoidable TG0 resets under load.
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t wdtResult = esp_task_wdt_init(&wdtConfig);
  if (wdtResult == ESP_ERR_INVALID_STATE) {
    // Arduino may already have initialized the TWDT. Reconfigure it rather
    // than silently accepting a different default timeout.
    wdtResult = esp_task_wdt_reconfigure(&wdtConfig);
  }
  Serial.printf("[WATCHDOG] core-3.x task-WDT config: %s (%d), timeout=20s\n",
                wdtResult == ESP_OK ? "OK" : "ERROR", (int)wdtResult);
  // v3.9.81: explicitly subscribe the Arduino loop task. Feed it at the
  // beginning of loop(); a genuinely wedged loop will then trip the TWDT.
  esp_err_t loopWdtResult = esp_task_wdt_add(NULL);
  if (loopWdtResult == ESP_ERR_INVALID_STATE) loopWdtResult = ESP_OK;
  Serial.printf("[WATCHDOG] loop task subscription: %s (%d)\n",
                loopWdtResult == ESP_OK ? "OK" : "ERROR", (int)loopWdtResult);

  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  Serial.printf("[INIT] WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  // v3.9.23: capture real WiFi disconnect reasons from the ESP32 event task.
  // The callback records only scalar state; detailed printing stays in loop().
  WiFi.onEvent(wifiEventCallback);
  Serial.println("[INIT] WiFi event diagnostics registered");

  // v3.9.25: Set WiFi power saving mode BEFORE any WiFi operations.
  // This must be called in setup() to avoid "wifi:sta is connecting, cannot set config"
  // error when calling WiFi.setSleep() during reconnection.
  // Call BEFORE WiFi.mode() to work around Arduino ESP32 library bug (Issue #12296).
  WiFi.setSleep(WIFI_SLEEP);
  Serial.printf("[INIT] WiFi power saving: %s\n", WIFI_SLEEP ? "ON" : "OFF");

  divider_ratio = R_DIVIDER_GND / (R_DIVIDER_SERIES + R_DIVIDER_GND);
  divider_scale = (R_DIVIDER_SERIES + R_DIVIDER_GND) / R_DIVIDER_GND;
  Serial.printf("[INIT] Divider: %.0fkΩ+%.0fkΩ ratio=%.4f scale=%.4f Vout@5V=%.3fV\n",
                R_DIVIDER_SERIES,R_DIVIDER_GND,divider_ratio,divider_scale,5.0f*divider_ratio);

  dataMutex = xSemaphoreCreateMutex();
  sensorDiagnosticsQueue = xQueueCreate(1, sizeof(SensorDiagnosticsSnapshot));
  sensorLogQueue = xQueueCreate(24, sizeof(SensorLogRecord));
  buzzerCommandQueue = xQueueCreate(8, sizeof(BuzzerCommand));
  if (sensorDiagnosticsQueue == NULL || sensorLogQueue == NULL ||
      buzzerCommandQueue == NULL) {
    Serial.println("[FATAL] Core-isolation queue creation failed");
  } else {
    Serial.println("[INIT] Core1->Core0 diagnostic/log queues ready (non-blocking)");
  }

  // v3.9.3: create the LEDC mutex before either Core can use the LEDC driver.
  // The sensor task is launched before the CO PWM setup below, so this must
  // exist before xTaskCreatePinnedToCore(sensorTask, ...).
  ledcMutex = xSemaphoreCreateMutex();
  if (ledcMutex == NULL) {
    Serial.println("[WARN] LEDC mutex creation failed — buzzer/CO PWM protected calls disabled");
  } else {
    Serial.println("[INIT] LEDC cross-core mutex created");
  }

  memset(&sd, 0, sizeof(sd));
  strncpy(sd.eng_msg, "Booting...", sizeof(sd.eng_msg));
  // Keep the reset cause in the normal engineering-message path so it is
  // also visible on the existing Blynk V19 diagnostic channel.
  engMsgf("ESP reset reason: %s - Firmware %s", bootResetReasonText, FIRMWARE_VERSION);
  // Pre-fill all sensor fields with UNAVAILABLE until first valid reads
  sd.co_ppm=sd.co_raw=sd.co_heater_v=SENSOR_UNAVAILABLE;
  sd.gps_unix_time = 0;
  sd.tvoc_ppb=sd.eco2_ppm=sd.aqi=SENSOR_UNAVAILABLE;
  sd.uvi=sd.sound_db=sd.dust_mg=SENSOR_UNAVAILABLE;
  sd.temp=sd.hum=SENSOR_UNAVAILABLE;
  sd.angle[0]=sd.angle[1]=sd.angle[2]=sd.imu_temp=SENSOR_UNAVAILABLE;
  // v2.3: pre-fill inertial-nav fields until origin is set from first GPS fix
  sd.pps_locked = false;
  sd.inav_state = INAV_STATE_NO_FIX_YET;
  sd.inav_lat = sd.inav_lng = SENSOR_UNAVAILABLE;
  sd.inav_alt = sd.inav_speed_mps = sd.inav_heading_deg = SENSOR_UNAVAILABLE;

  sensorSnapshotQueue = xQueueCreate(1, sizeof(SensorData));
  if (sensorSnapshotQueue == NULL) {
    Serial.println("[FATAL] Sensor snapshot queue creation failed");
  } else {
    xQueueOverwrite(sensorSnapshotQueue, &sd);
    Serial.println("[INIT] One-way Core1 -> Core0 sensor snapshot queue ready");
  }

  telemetryPairQueue = xQueueCreate(1, sizeof(Core1TelemetryPair));
  connectionSnapshotQueue = xQueueCreate(1, sizeof(Core0ConnectionSnapshot));
  if (telemetryPairQueue == NULL || connectionSnapshotQueue == NULL) {
    Serial.println("[FATAL] Coherent telemetry boundary queue creation failed");
  } else {
    Core0ConnectionSnapshot initialConnection = {};
    xQueueOverwrite(connectionSnapshotQueue, &initialConnection);
    Serial.println("[INIT] Core1-sealed V61/V62 pair queues ready");
  }

  // ── GPIO setup ───────────────────────────────────────────────────────────
#if 0  // v3.9.150: moved to sensorHardwareInit() on Core 1.
  pinMode(ledPin11,OUTPUT); digitalWrite(ledPin11,LOW);
  pinMode(ledPin12,OUTPUT); digitalWrite(ledPin12,LOW);
  pinMode(ledPin13,OUTPUT); digitalWrite(ledPin13,LOW);
  pinMode(PPS_LED_PIN, OUTPUT); setPpsLed(false);
  pinMode(dustLED, OUTPUT); digitalWrite(dustLED, HIGH);
  pinMode(buzzPin, OUTPUT);
  // v3.9.4: attach the buzzer ONCE during single-threaded setup, before
  // sensorTask starts on Core 0. Use channel 8 (LEDC group 1), deliberately
  // separate from the CO heater's channel 0 (LEDC group 0).
  buzzerLEDCReady = ledcAttachChannelSafe(buzzPin, 5000, 8, LEDC_CHANNEL_BUZZER);
  if (buzzerLEDCReady) {
    ledcWriteSafe(buzzPin, 0);
    Serial.println("[INIT] Buzzer LEDC attached on channel 8 (group 1)");
  } else {
    Serial.println("[WARN] Buzzer LEDC attach failed — buzzer disabled");
  }
  pinMode(CO_ADC_PIN,   INPUT);
  pinMode(CO_A1_PIN,    INPUT);
  pinMode(UV_OUT_PIN,   INPUT);
  pinMode(UV_REF_PIN,   INPUT);
  pinMode(DUST_ADC_PIN, INPUT);

  // ── ADC configuration ─────────────────────────────────────────────────────
  // Keep the existing per-pin attenuation calls. They are the current Arduino
  // 3.x API and avoid the old global attenuation path that was problematic when
  // this project still used the legacy I2S driver.
  analogReadResolution(12);
  // Set 0–3.3V range (ADC_11db) individually on every ADC1 pin we use.
  // Do NOT call analogSetAttenuation() — it conflicts with I2S legacy driver.
  analogSetPinAttenuation(CO_ADC_PIN,   ADC_11db);
  analogSetPinAttenuation(CO_A1_PIN,    ADC_11db);
  analogSetPinAttenuation(UV_OUT_PIN,   ADC_11db);
  analogSetPinAttenuation(UV_REF_PIN,   ADC_11db);
  analogSetPinAttenuation(DUST_ADC_PIN, ADC_11db);
  Serial.println("[INIT] ADC configured (12-bit, per-pin 11dB)");

  // ── I2C for ENS160+AHT2x+BMI160 ──────────────────────────────────────────
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // SDA=GPIO21, SCL=GPIO22
  // v3.9.32: one-time bus scan. This is diagnostic only and does not replace
  // the normal per-sensor preflight checks.
  Serial.println("[I2C DIAG] Startup scan: 0x38(AHT2x), 0x53(ENS160), 0x68(BMI160), 0x69(BMI160)");
  for (uint8_t diagAddr = 0x08; diagAddr <= 0x77; ++diagAddr) {
    if (diagAddr == 0x38 || diagAddr == 0x53 || diagAddr == 0x68 || diagAddr == 0x69) {
      uint32_t diagStartUs = micros();
      bool present = i2cDevicePresent(diagAddr);
      DBG_PRINTF("[I2C DIAG] scan 0x%02X -> %s (%lums err=%u SDA=%d SCL=%d)\n",
                 diagAddr, present ? "ACK" : "NO ACK",
                 (unsigned long)((micros() - diagStartUs) / 1000UL),
                 i2cLastWireError, i2cLastSdaLevel, i2cLastSclLevel);
    }
  }
  // CRITICAL: without a bounded timeout, Wire.requestFrom()/endTransmission()
  // can block indefinitely if a device doesn't ACK (e.g. BMI160 not yet
  // wired, floating SDA/SCL, bad joint). 50ms bounds every Wire call for the
  // rest of the sketch — belt-and-suspenders alongside the fix below.
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  // ── ENS160+AHT2x / BMI160 init — DELIBERATELY NOT DONE HERE ──────────────
  // Earlier versions called initENS()/initBMI160() directly in setup(), which
  // runs entirely on Core 1 BEFORE wifiConnect() below. If either device
  // failed to ACK on the bus (not yet wired, floating SDA/SCL, bad joint),
  // the I2C call could block long enough for the ESP32's own system
  // watchdog (TG0WDT — separate from the app-level task/interrupt
  // watchdogs discussed elsewhere in this file) to fire, producing an
  // endless "rst:0x7 (TG0WDT_SYS_RESET)" boot loop that never reached WiFi.
  // The Wire.setTimeOut(50) above bounds that specific hang, but the more
  // fundamental fix is architectural: I2C sensor init now happens EXCLUSIVELY
  // inside sensorTask (Core 0) — see the ENS160 block in sensorTask and the
  // BMI160 lazy-init block at the top of inertialNav(), both of which already
  // attempt initialisation on their very first call and retry periodically
  // thereafter if the device is absent. This guarantees WiFi on Core 1 can
  // NEVER be blocked by I2C, no matter what is or isn't wired to the bus —
  // matching the same principle already used for the CO sensor: nothing that
  // can stall waiting on hardware runs before WiFi starts.

  // ── GPS 1PPS interrupt (v2.3) ─────────────────────────────────────────────
  if (PPSsensorThere) {
    pinMode(PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsISR, RISING);
    Serial.println("[INIT] GPS PPS interrupt attached (GPIO5)");
  }

  // ── CO LEDC PWM ──────────────────────────────────────────────────────────
  if (COsensorThere) {
    // v3.9.4: explicit channel 0 keeps the CO heater in LEDC group 0.
    ledcAttachChannelSafe(CO_PWM_PIN, LEDC_FREQ_CO, LEDC_RES_CO, LEDC_CHANNEL_CO);
    ledcWriteSafe(CO_PWM_PIN, 0);
  }

  // [SUPERSEDED v2.4] JY-901 UART2 init removed — GPIO16/17 are free. BMI160
  // (the only IMU now) is initialised in the ENS160/BMI160 block below.

  // ── GPS — ATGM336H NMEA only, 9600 baud ──────────────────────────────────
  initGPS();

  // ── OLED (optional, shares I2C) ──────────────────────────────────────────
  if (DISPLAYON) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
      Serial.println("[WARN] OLED init failed");
    else { display.setTextColor(WHITE); display.clearDisplay(); display.display(); }
  }

  randomSeed(analogRead(0));

  esp_task_wdt_reset();  // v3.9.127: feed before I2S init, in case time was already spent in GPS/OLED init above.

  // ── INMP441 I2S ───────────────────────────────────────────────────────────
  // Arduino-ESP32 3.x uses the ESP-IDF 5.x standard I2S driver. It is independent
  // of the ADC driver, so the old legacy-I2S ordering restriction no longer applies.
  if (INMPsensorThere) initINMP441();
#endif

  // v3.9.127 FIX: field logs showed a TG1WDT_SYS_RESET (idle-task watchdog)
  // during setup(), with the last successfully-printed line being INMP441's
  // own "I2S mic OK" success message — i.e. i2s_new_channel()/
  // i2s_channel_init_std_mode()/i2s_channel_enable() all returned ESP_OK,
  // and the reset happened somewhere in this stretch or the three
  // xTaskCreatePinnedToCore() calls immediately following it. setup() runs
  // on Core 1 with no yield point across this whole sequence, so if I2S
  // driver install (heap allocation + hardware register configuration)
  // combined with three back-to-back task-creation calls (each of which
  // briefly touches scheduler internals) ran long enough without letting
  // Core 1's idle task run, the idle watchdog can fire even though nothing
  // is actually wedged — a false-positive reset from a legitimately slow
  // but finite stretch of setup(). Feeding the watchdog explicitly here
  // (setup()'s task is already subscribed via esp_task_wdt_add(NULL) a few
  // lines up) removes that as a possibility without changing what any of
  // these calls actually do.
  esp_task_wdt_reset();

  // ── Launch PPS LED task ───────────────────────────────────────────────────
  // v3.6: Keep the PPS LED timing independent from sensorTask. The task is
  // woken directly by the GPIO5 PPS ISR and produces one fixed 250ms pulse.
  xTaskCreatePinnedToCore(ppsLedTask, "ppsLedTask", 2048, NULL, 2,
                          &ppsLedTaskHandle, 1);
  esp_task_wdt_reset();

  // LEDC/buzzer servicing is also kept off the communications core.
  xTaskCreatePinnedToCore(sensorOutputTask, "sensorOutput", 2048, NULL, 1, NULL, 1);
  esp_task_wdt_reset();

  // ── Launch sensor task on Core 0 ─────────────────────────────────────────
  // Core 0 is reserved for the Arduino communications loop, WiFi events and
  // the core-pinned ESP-IDF WiFi/lwIP tasks. Sensor driver initialization and
  // all periodic sensor I/O are owned by this Core-1 task.
  const BaseType_t sensorTaskCreateResult =
      xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, NULL, 1, NULL, 1);
  Serial.printf("[TASK] communications setup/loop Core %d; sensorTask Core 1 create=%s\n",
                xPortGetCoreID(), sensorTaskCreateResult == pdPASS ? "OK" : "FAILED");
  esp_task_wdt_reset();

  Serial.printf("[TASK] coherent telemetry: Core 1 snapshot producer only; Core 0 owns nonblocking TCP (loop core=%d)\n",
                xPortGetCoreID());
  esp_task_wdt_reset();

  // v3.9.18: independent Core-1 monitor. This task only observes loop progress;
  // it never resets the ESP32 and therefore cannot mask the original failure.
  xTaskCreatePinnedToCore(core1StallMonitorTask, "commsMon", 3072, NULL, 2, NULL, 1);
  esp_task_wdt_reset();

  // ── WiFi + Blynk config ──────────────────────────────────────────────────
  // Start WiFi asynchronously. The previous wifiConnect() helper remains
  // available, but normal operation uses the non-blocking state machine so
  // Core 1 remains responsive while the station associates with the AP.
  if (WIFI) {
    lastWiFiCheck = millis() - WIFI_RECONNECT_MS;
    wifiStartConnect();
  }

  // ── CO sensor startup ────────────────────────────────────────────────────
#if 0  // v3.9.150: CO startup moved to sensorHardwareInit() on Core 1.
  if (COsensorThere) {
    sens_val = sensor_reading_clean_air;
    if (!coSensorPresent()) {
      Serial.println("[WARN] CO sensor not detected (GPIO36 floating)");
      engMsg("CO: sensor absent — check A0 wiring and divider");
      if (dataMutexTake(20)==pdTRUE) {
        sd.co_no_sensor=true; sd.status_flags|=STATUS_CO_NO_SNS;
        sd.co_ppm=SENSOR_UNAVAILABLE; xSemaphoreGive(dataMutex);
      }
    } else {
      engMsg("CO: sensor detected — starting closed-loop heater control");
    }
    startMeasurementPhase();
  }
#endif

  // ── BlynkTimer intervals ──────────────────────────────────────────────────
  // v3.9.180 prevention profile: V61/V62 retain a coherent 2s snapshot, while
  // dashboard and diagnostic pins are refreshed
  // one at a time instead of in bursts.
  // V61/V62 are produced on Core 1 and published by the Core-0 telemetry state machine.
  blynkTimer.setInterval(BLYNK_V4_INTERVAL_MS, blynkSendV4);
  blynkTimer.setInterval(BLYNK_DASHBOARD_ROUNDROBIN_MS, blynkSendDashboardRoundRobin);
  blynkTimer.setInterval(BLYNK_DIAG_ROUNDROBIN_MS, blynkSendDiagnosticsRoundRobin);
  blynkTimer.setInterval(MAP_SEND_INTERVAL_MS, blynkSendMap);
  engMsgf("Setup OK v%s - Core-0 Blynk + nonblocking HTTP; Core-1 sensors only",
          FIRMWARE_VERSION);
  Serial.printf("[Blynk DATA PLANE] V61/V62 persistent nonblocking HTTP on Core 0; record=%lums dashboard=%lums diagnostic=%lums V4=%lums pacing=%lums\n",
                (unsigned long)BLYNK_COHERENT_RECORD_MS,
                (unsigned long)BLYNK_DASHBOARD_ROUNDROBIN_MS,
                (unsigned long)BLYNK_DIAG_ROUNDROBIN_MS,
                (unsigned long)BLYNK_V4_INTERVAL_MS,
                (unsigned long)BLYNK_TX_INTERVAL_MS);
#if ESP32_RX_ISOLATION_TEST
  Serial.println("[INIT] ESP32_RX_ISOLATION_TEST=1 — heavy sensor workload bypassed for RX isolation test");
#else
  Serial.println("[INIT] ESP32_RX_ISOLATION_TEST=0 — full sensor workload active");
#endif
  setupDone = true;
}

// Strictly non-blocking TCP reachability probe, hard-bounded by select().
// WiFiClient::setTimeout() only reliably bounds READ operations on the
// ESP32 Arduino core — the connect() phase itself can still block far
// longer than requested when the host is unreachable or the handshake
// hangs. That unbounded connect() is exactly what can trip the CPU1
// Interrupt WDT (~300ms) even though BLYNK_SERVER_PROBE_TIMEOUT_MS is set.
//
// v3.9.24 REWRITE — NON-BLOCKING STATE MACHINE:
// The previous implementation called lwip_select() with a timeout of up to
// several seconds, and since this ran on Core 1 — the SAME core that must
// keep servicing the live Blynk TCP session via Blynk.run() — that select()
// call stalled the entire loop() for its full duration. Every millisecond
// spent blocked in select() was a millisecond during which any inbound
// Blynk server response (including PING replies) went unread. This was
// confirmed as the direct cause of "Heartbeat timeout" disconnects even
// while the underlying WiFi/TCP link was otherwise healthy: the probe was
// starving the very socket it was trying to help recover.
//
// This rewrite splits the probe into three non-blocking steps, each of
// which is polled once per loop() pass and returns immediately:
//   1. blynkProbeStart()  — create the socket, kick off connect(), return.
//   2. blynkProbePoll()   — check (non-blocking) whether connect() finished.
//   3. blynkProbeAbort()  — close the socket if the caller gives up early.
// No call in this file blocks for more than a socket() or connect() syscall,
// both of which are O(microseconds) on lwIP for a non-blocking socket.
struct BlynkProbeState {
  int sock = -1;
  unsigned long startedAt = 0;
  bool active = false;
};
static BlynkProbeState blynkProbe;

// Begin a new non-blocking TCP probe. Returns false immediately if the
// socket/connect could not even be started (treated as an instant failure
// by the caller); otherwise the probe is now "active" and must be polled
// via blynkProbePoll() on subsequent loop() passes.
bool blynkProbeStart(const char* host, uint16_t port)
{
  if (blynkProbe.active) return true;  // already running — nothing to do

  IPAddress ip;
  // BLYNK_SERVER is a literal dotted-decimal IP, so parsing is pure string
  // work and cannot block. See historical note below on why hostByName()
  // is avoided here.
  if (!ip.fromString(host)) {
    if (!WiFi.hostByName(host, ip)) return false;
  }

  int sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) return false;

  int flags = lwip_fcntl(sock, F_GETFL, 0);
  lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port);
  addr.sin_addr.s_addr = (uint32_t)ip;

  int res = lwip_connect(sock, (struct sockaddr*)&addr, sizeof(addr));
  if (res == 0) {
    // Connected immediately (rare, e.g. loopback) — still go through the
    // active/poll path so the caller's state machine only has one exit.
    blynkProbe.sock = sock;
    blynkProbe.startedAt = millis();
    blynkProbe.active = true;
    return true;
  }
  if (errno == EINPROGRESS) {
    blynkProbe.sock = sock;
    blynkProbe.startedAt = millis();
    blynkProbe.active = true;
    return true;
  }

  // connect() failed synchronously (e.g. immediate ECONNREFUSED/ENETUNREACH).
  lwip_close(sock);
  return false;
}

// Poll an in-progress probe. Call this once per loop() pass while
// blynkProbe.active is true. Uses select() with a ZERO timeout, so this
// call always returns within microseconds regardless of connect state —
// it never blocks Core 1.
//   done  = true once the probe has a final answer (success, failure, or
//           timeoutMs exceeded)
//   ok    = only meaningful when done == true; true if the TCP connect
//           actually succeeded
bool blynkProbePoll(uint32_t timeoutMs, bool* done)
{
  *done = false;
  if (!blynkProbe.active) { *done = true; return false; }

  fd_set wfds; FD_ZERO(&wfds); FD_SET(blynkProbe.sock, &wfds);
  struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;  // zero timeout: poll only
  int sel = lwip_select(blynkProbe.sock + 1, NULL, &wfds, NULL, &tv);

  if (sel > 0) {
    int soErr = 0; socklen_t len = sizeof(soErr);
    lwip_getsockopt(blynkProbe.sock, SOL_SOCKET, SO_ERROR, &soErr, &len);
    lwip_close(blynkProbe.sock);
    blynkProbe.active = false;
    *done = true;
    return (soErr == 0);
  }

  if (millis() - blynkProbe.startedAt >= timeoutMs) {
    // Overall probe timeout exceeded across however many loop() passes it
    // took to get here — give up, but this never cost more than a few
    // microseconds of Core 1 time on any single pass.
    lwip_close(blynkProbe.sock);
    blynkProbe.active = false;
    *done = true;
    return false;
  }

  // Still waiting — not done, caller should poll again next loop() pass.
  return false;
}

// Abandon an in-progress probe (e.g. caller state machine is resetting for
// an unrelated reason). Safe to call even if no probe is active.
void blynkProbeAbort()
{
  if (blynkProbe.active) {
    lwip_close(blynkProbe.sock);
    blynkProbe.active = false;
  }
}

// v3.9.17: Explicit Blynk protocol heartbeat.
//
// Why this exists:
// Blynk v0.6.1 normally sends PING when either incoming OR outgoing activity
// has been quiet for BLYNK_HEARTBEAT seconds. Our telemetry queue keeps
// outgoing traffic active, while the failure we observed was specifically a
// lack of incoming traffic: after ~63s the library printed "Heartbeat timeout"
// even though WiFi still reported WL_CONNECTED. That means the TCP/WiFi path
// was associated but no Blynk response reached the protocol layer.
//
// Sending a small protocol PING every 20s gives the LAN/server path a frequent
// request/response opportunity. PING is explicitly exempt from BLYNK_MSG_LIMIT
// in v0.6.1, so this cannot recreate the telemetry-burst problem.
// v3.9.42 FIX: removed the RX-staleness watchdog added in v3.9.40. It set
// blynkForceReconnect=true purely from "no bytes read in N seconds", but that
// can trip on a perfectly healthy, already-connected session (server just had
// nothing new to send and a PONG lagged past the threshold). Once tripped, the
// handshake-service code below re-enters its handshake path on an ALREADY-
// CONNECTED session and, if that doesn't resolve within
// BLYNK_HANDSHAKE_TIMEOUT_MS, calls Blynk.disconnect() + WiFi.disconnect() —
// a fully self-inflicted disconnect. This is what produced the
// "WiFi: reconnect (reason=8)" (ASSOC_LEAVE) log lines roughly 3 minutes after
// every successful connect: reason 8 is what the driver reports for OUR OWN
// WiFi.disconnect() call, not an external kick or weak signal. The pre-
// existing blynkProbeFailCount / blynkHandshakeFailCount escalation logic
// already exists to detect and recover from a genuinely dead session; that is
// kept, and is now serviced faster via the event-driven WiFi reconnect
// (v3.9.41). No separate staleness heuristic is layered on top of it.
static void blynkHeartbeatService()
{
  if (!Blynk.connected()) return;

  const unsigned long now = millis();
  if (now - lastBlynkForcePing < BLYNK_STOCK_TX_PING_INTERVAL_MS) return;

  const unsigned long pingStart = millis();
  // v3.9.132 DIAGNOSTIC: field investigation established that "heartbeat
  // lost in some layer" cannot mean a server-initiated PING going missing —
  // stock BlynkProtocol::run() never has the SERVER proactively ping the
  // client; the client always pings the server (this call), and the
  // server's RESPONSE to that specific PING is what resets lastActivityIn.
  // serverPingPeekObs has read 0 in every field log, meaning no inbound
  // PING-shaped frame was ever observed — consistent with there being none
  // to observe. What was never directly measured: whether THIS device-sent
  // PING's own RESPONSE comes back, and how long it takes, tracked by its
  // actual msgId rather than inferred from general RX traffic. Rotate through
  // the reserved high range so consecutive RESPONSE headers are distinguishable
  // by the RX observer while the RESPONSE handler in BlynkProtocol.h can still
  // correlate each specific reply via blynkPingRttWatch below.
  static uint16_t blynkHeartbeatPingMsgId = 0xF000;
  ++blynkHeartbeatPingMsgId;
  if (blynkHeartbeatPingMsgId < 0xF001) {
    blynkHeartbeatPingMsgId = 0xF001;
  }
  const uint16_t pingMsgId = blynkHeartbeatPingMsgId;
  blynkPingRttWatchArm(pingMsgId, pingStart);
  core1StageBegin(CORE1_STAGE_BLYNK_HEARTBEAT);
  Blynk.sendCmd(BLYNK_CMD_PING, pingMsgId);
  core1StageEnd();
  // If backpressure is active, this exact PING becomes the end-to-end proof.
  // Its RESPONSE cannot precede older bytes in the ordered TCP stream.
  #if CONNECTION_MODE == 1
    _blynkWifiClient.armPeerProbe(pingMsgId);
  #endif
  const unsigned long pingElapsed = millis() - pingStart;
  if (pingElapsed > BLYNK_RUN_GUARD_MS) diagBlynkRunSlow++;
  perfRecordMax(perfMaxBlynkRunMs, pingElapsed);
  if (!Blynk.connected()) {
    // sendCmd() detected a failed write and already tore the library's
    // connection state down. Bring the sketch's own session tracking back
    // into sync immediately instead of waiting for the next Blynk.run()
    // pass to notice — that gap is exactly what let TX keep queuing
    // against a session the library had already abandoned.
    DBG_PRINTF("[Blynk PING] sendCmd disconnected the session (elapsed=%lums) — syncing session state\n",
               pingElapsed);
    blynkStockRxForensicSessionEnd(millis(), "heartbeat-sendcmd-disconnected");
    blynkRecordSessionLossAndMaybeRestart("heartbeat-sendcmd-disconnected");
    blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
    blynkStockStateSinceMillis = millis();
    blynkOfflineSince = millis();
    blynkStockNextRunMillis = millis() + BLYNK_STOCK_RETRY_INTERVAL_MS;
    blynkStockTxRecoveryPending = false;
    blynkTxDropCoherentPair();
    return;
  }

  // The PING write completed and the protocol still reports connected. Record
  // it even when it was slow so the backoff path cannot immediately retry the
  // same diagnostic write as soon as the reachability probe succeeds again.
  lastBlynkForcePing = millis();
  blynkLastForcePingMillis = lastBlynkForcePing;
  blynkForcePingCount++;
  blynkStockTxPingQuietUntil = lastBlynkForcePing + BLYNK_STOCK_TX_PING_SETTLE_MS;

  if (pingElapsed >= BLYNK_STOCK_TX_STALL_MS) {
    // Slow does not necessarily mean failed. Pause application telemetry and
    // let Blynk.run() determine session health from real protocol/TCP state.
    DBG_PRINTF("[Blynk PING] slow stock write=%lums — application TX backoff; session kept\n", pingElapsed);
    blynkStockEnterTxQuarantine(5, pingElapsed);
    return;
  }

  DBG_PRINTF("[Blynk] Heartbeat PING sent #%lu duration=%lums\n",
             (unsigned long)blynkForcePingCount, pingElapsed);
}


// v3.9.117: stock-Blynk application-TX reachability gate. This probe is NOT
// the Blynk transport and never carries Blynk protocol data. It is a separate
// non-blocking TCP connect used only to prevent application telemetry from
// entering stock WiFiClient::write() while the configured endpoint is plainly
// unreachable. The Blynk session itself remains entirely stock.
static void blynkStockTxProbeService(unsigned long now)
{
#if CONNECTION_MODE == 1
  if (!Blynk.connected() || WiFi.status() != WL_CONNECTED) {
    if (blynkProbeInFlight) blynkProbeAbort();
    blynkStockTxProbeInFlight = false;
    blynkStockLastTxProbeOkMillis = 0;
    return;
  }

  if (blynkProbeInFlight) {
    bool done = false;
    const bool ok = blynkProbePoll(BLYNK_STOCK_TX_PROBE_TIMEOUT_MS, &done);
    if (!done) return;
    blynkProbeInFlight = false;
    blynkStockTxProbeInFlight = false;
    if (ok) {
      blynkStockLastTxProbeOkMillis = millis();
      blynkStockTxProbeFailures = 0;
      blynkStockNextTxProbeMillis = millis() + BLYNK_STOCK_TX_PROBE_INTERVAL_MS;
      ++diagServerProbeSuccess;
    } else {
      ++diagServerProbeFails;
      if (blynkStockTxProbeFailures < 255U) ++blynkStockTxProbeFailures;
      blynkStockLastTxProbeOkMillis = 0;
      blynkStockNextTxProbeMillis = millis() + 1000UL;
      DBG_PRINTF("[Blynk TX GATE] independent TCP probe failed (%u/%u) — application TX held\n",
                 (unsigned)blynkStockTxProbeFailures,
                 (unsigned)BLYNK_STOCK_TX_PROBE_FAILS_BEFORE_REFRESH);
      if (blynkStockTxProbeFailures >= BLYNK_STOCK_TX_PROBE_FAILS_BEFORE_REFRESH) {
        const unsigned long recoverAt = millis();
        blynkStockTxProbeFailures = 0;
        blynkStockTxQuarantineUntil = recoverAt + BLYNK_STOCK_TX_QUARANTINE_MS;
        blynkStockStableUntilMillis = blynkStockTxQuarantineUntil;
        blynkTxDropNonPriority();
        blynkTxDropCoherentPair();
        blynkTxGraceUntil = recoverAt + BLYNK_STOCK_TX_QUARANTINE_MS;
        DBG_PRINTLN("[Blynk TX GATE] repeated probe failure — application TX remains held; live Blynk session kept");
      }
    }
    return;
  }

  if ((blynkStockLastTxProbeOkMillis == 0 ||
       (uint32_t)(now - blynkStockLastTxProbeOkMillis) >= BLYNK_STOCK_TX_PROBE_INTERVAL_MS) &&
      (blynkStockNextTxProbeMillis == 0 || (int32_t)(now - blynkStockNextTxProbeMillis) >= 0)) {
    if (blynkProbeStart(BLYNK_SERVER, BLYNK_PORT)) {
      ++diagServerProbeStarts;
      blynkProbeInFlight = true;
      blynkStockTxProbeInFlight = true;
    } else {
      blynkStockTxProbeInFlight = false;
      blynkStockLastTxProbeOkMillis = 0;
      blynkStockNextTxProbeMillis = now + 1000UL;
      if (blynkStockTxProbeFailures < 255U) ++blynkStockTxProbeFailures;
    }
  }
#endif
}


// ============================================================================
// LOOP — Core 0: WiFi + Blynk only
// All sensor I/O and sensor-driver initialization are on Core 1.
// Blynk.run() in loop() is the critical design decision that prevents
// the IWDT crash seen in all v1.x versions (where Blynk ran in a FreeRTOS
// task and Blynk.connect(timeout) spun for 5s blocking interrupt handlers).
// ============================================================================

// ============================================================================
// LOCAL BLYNK-SERVER ARP REFRESH (Core 0 -> lwIP TCP/IP thread)
//
// lwIP's dynamic ARP entry lifetime is 300 seconds. The server is on the local
// subnet, and the field failure begins at ~306 seconds even though association,
// RSSI and the ESP32 IP address remain valid. Requesting the server mapping once
// per minute keeps that single entry fresh and gives an explicit result from
// the layer immediately below IP/TCP.
// ============================================================================
#define BLYNK_ARP_REFRESH_INTERVAL_MS 60000UL

struct BlynkArpRefreshContext {
  ip4_addr_t target;
  volatile bool pending;
  volatile uint32_t completedSequence;
  volatile int requestResult;
  volatile int entryBefore;
};

static BlynkArpRefreshContext blynkArpRefreshContext = {};
static uint32_t blynkArpRefreshLastRequestMs = 0;
static uint32_t blynkArpRefreshRequestSequence = 0;
static uint32_t blynkArpRefreshReportedSequence = 0;

static void blynkArpRefreshInTcpipThread(void *argument)
{
  BlynkArpRefreshContext *ctx =
      static_cast<BlynkArpRefreshContext *>(argument);
  struct netif *route = ip4_route(&ctx->target);
  if (route == NULL) {
    ctx->entryBefore = -1;
    ctx->requestResult = ERR_RTE;
  } else {
    struct eth_addr *mac = NULL;
    const ip4_addr_t *cachedIp = NULL;
    ctx->entryBefore = (int)etharp_find_addr(route, &ctx->target,
                                             &mac, &cachedIp);
    ctx->requestResult = (int)etharp_request(route, &ctx->target);
  }
  ctx->completedSequence = blynkArpRefreshRequestSequence;
  ctx->pending = false;
}

static void blynkArpRefreshService(unsigned long now)
{
  if (blynkArpRefreshContext.completedSequence !=
      blynkArpRefreshReportedSequence) {
    blynkArpRefreshReportedSequence =
        blynkArpRefreshContext.completedSequence;
    Serial.printf("[Blynk ARP REFRESH] seq=%lu cachedEntry=%d requestResult=%d target=%s\n",
                  (unsigned long)blynkArpRefreshReportedSequence,
                  blynkArpRefreshContext.entryBefore,
                  blynkArpRefreshContext.requestResult, BLYNK_SERVER);
  }

  if (WiFi.status() != WL_CONNECTED) {
    blynkArpRefreshLastRequestMs = 0;
    return;
  }
  if (blynkArpRefreshContext.pending) return;
  if (blynkArpRefreshLastRequestMs != 0 &&
      (uint32_t)(now - blynkArpRefreshLastRequestMs) <
          BLYNK_ARP_REFRESH_INTERVAL_MS) return;

  IPAddress serverIp;
  if (!serverIp.fromString(BLYNK_SERVER)) {
    // If BLYNK_SERVER becomes a DNS name, do not guess its resolved address.
    return;
  }

  IP4_ADDR(&blynkArpRefreshContext.target,
           serverIp[0], serverIp[1], serverIp[2], serverIp[3]);
  blynkArpRefreshContext.pending = true;
  blynkArpRefreshLastRequestMs = now;
  ++blynkArpRefreshRequestSequence;
  const err_t callbackResult =
      tcpip_try_callback(blynkArpRefreshInTcpipThread,
                         &blynkArpRefreshContext);
  if (callbackResult != ERR_OK) {
    blynkArpRefreshContext.pending = false;
    blynkArpRefreshContext.requestResult = callbackResult;
    blynkArpRefreshContext.completedSequence =
        blynkArpRefreshRequestSequence;
  }
}

static inline bool blynkRunGuardCheck(uint32_t elapsedMs, const char* path)
{
  if (elapsedMs <= BLYNK_RUN_GUARD_MS) return false;

  blynkRunGuardCount++;
  blynkRunLastGuardMs = elapsedMs;
  if (elapsedMs > blynkRunMaxGuardMs) blynkRunMaxGuardMs = elapsedMs;

  Serial.printf("[Blynk] RUN DIAG: %s took %lums (guard=%lums timeout=%lums) count=%lu WiFi=%s connected=%d heap=%u\n",
                path, (unsigned long)elapsedMs,
                (unsigned long)BLYNK_RUN_GUARD_MS,
                (unsigned long)BLYNK_TIMEOUT_MS,
                (unsigned long)blynkRunGuardCount,
                WiFi.status() == WL_CONNECTED ? "OK" : "OFF",
                Blynk.connected() ? 1 : 0,
                ESP.getFreeHeap());

  // A long Blynk.run() is a transport timing event, not proof that WiFi is
  // broken. In STOCK mode, do NOT call Blynk.disconnect() here: doing so can
  // turn a slow server response into a self-inflicted reconnect loop. Instead
  // back off the next disconnected Blynk.run() attempt.
#if CONNECTION_MODE == 0
  blynkHandshakeActive = false;
  blynkForceReconnect = false;
  blynkOfflineSince = millis();
  lastBlynkConnectAttempt = millis();
  Blynk.disconnect();
  _stockBlynkTransportDiagnostics.disconnect();
#else
  if (!Blynk.connected()) {
    blynkOfflineSince = millis();
    blynkStockNextRunMillis = millis() + BLYNK_STOCK_RETRY_INTERVAL_MS;
  }
#endif
  return true;
}

static void telemetryNetworkUpdateBlynkGate()
{
  const uint32_t now = millis();
  // An application checkpoint and a protocol PING are the same end-to-end
  // ordered-stream proof. While the former is pending BlynkProtocol does not
  // send the latter, so watching only protocolPingPending() left HTTP active
  // throughout every v3.9.213 failure. Quiesce the bulk stream for either
  // exact proof and leave the Blynk socket alone to retransmit.
  const bool exactApplicationAckOverdue =
      Blynk.connected() &&
      blynkStockSessionState == BLYNK_STOCK_SESSION_ONLINE &&
      Blynk.applicationAckPending() && Blynk.applicationAckAge() >= 3000UL;
  const bool exactProtocolPingOverdue =
      Blynk.connected() &&
      blynkStockSessionState == BLYNK_STOCK_SESSION_ONLINE &&
      Blynk.protocolPingPending() && Blynk.protocolPingAge() >= 3000UL;
  const bool exactProbeOverdue =
      exactApplicationAckOverdue || exactProtocolPingOverdue;

  if (exactProbeOverdue) {
    if (!telemetryHttpHealthThreatActive) {
      telemetryHttpHealthThreatActive = true;
      telemetryHttpHealthPauseLatched = true;
      ++telemetryHttpHealthIsolationCount;
      Serial.printf("[Blynk/HTTP ISOLATION] exact Blynk proof overdue; closing HTTP PCB appPending=%d appId=%u appAge=%lums protocolPending=%d pingId=%u pingAge=%lums count=%lu\n",
                    Blynk.applicationAckPending() ? 1 : 0,
                    (unsigned)Blynk.applicationAckPingId(),
                    (unsigned long)Blynk.applicationAckAge(),
                    Blynk.protocolPingPending() ? 1 : 0,
                    (unsigned)Blynk.protocolPingId(),
                    (unsigned long)Blynk.protocolPingAge(),
                    (unsigned long)telemetryHttpHealthIsolationCount);
    }
  } else if (telemetryHttpHealthThreatActive) {
    telemetryHttpHealthThreatActive = false;
    telemetryHttpHealthResumeNotBeforeMs =
        now + TELEMETRY_HTTP_HEALTH_HOLDOFF_MS;
    Serial.printf("[Blynk/HTTP ISOLATION] exact Blynk proof recovered; starting %lums stable quiet window\n",
                  (unsigned long)TELEMETRY_HTTP_HEALTH_HOLDOFF_MS);
  }

  if (telemetryHttpHealthPauseLatched &&
      !telemetryHttpHealthThreatActive &&
      (int32_t)(now - telemetryHttpHealthResumeNotBeforeMs) >= 0) {
    telemetryHttpHealthPauseLatched = false;
    Serial.println("[Blynk/HTTP ISOLATION] stable quiet window complete; bulk HTTP may resume");
  }

  telemetryHttpPauseRequested =
      sharedPathRecoveryState != SHARED_PATH_IDLE ||
      !Blynk.connected() || exactProbeOverdue ||
      telemetryHttpHealthPauseLatched;
}

void loop()
{
  // Independent Core-1 supervisor heartbeat. Update before any queue, Serial,
  // WiFi or Blynk call so a stall anywhere in this loop remains observable.
  core1LastProgressUs = micros();
  ++wdtForensicLoopCount;
  wdtForensicHeartbeat();
  // v3.9.81: feed the explicitly subscribed Arduino loop task immediately.
  esp_task_wdt_reset();
  core0DrainSensorLog();
  core0RefreshSensorDiagnostics();
  core0PublishConnectionSnapshot(false);

  loopState = LOOP_STATE_RUN;
  static uint32_t perfLastLoopStartUs = 0;
  uint32_t perfLoopStartUs = micros();
  core1WifiConnectedSnapshot = (WiFi.status() == WL_CONNECTED);
  core1BlynkConnectedSnapshot = Blynk.connected();
#if CONNECTION_MODE == 1
  blynkApplyNoDelayOnConnectEdge();
#endif
  if (perfLastLoopStartUs != 0)
    perfRecordMax(perfMaxLoopGapMs,
                  (uint32_t)((perfLoopStartUs - perfLastLoopStartUs) / 1000UL));
  perfLastLoopStartUs = perfLoopStartUs;


  if (WIFI) {
    loopState = LOOP_STATE_WIFI;
    core1StageBegin(CORE1_STAGE_WIFI_SERVICE);
    wifiService();
    core1StageEnd();

    // Shared-path recovery owns all network activity until the radio has been
    // rebuilt and a fresh GOT_IP has arrived. v3.9.204 returned only from
    // wifiService(), allowing the Blynk block below to start another one-second
    // connect during quiesce. Return from loop() itself so neither Blynk nor
    // HTTP control code can race the recovery state machine. POST_IP_GRACE is
    // intentionally excluded: Blynk may reconnect then while HTTP stays paused.
    if (sharedPathRecoveryState == SHARED_PATH_QUIESCE ||
        sharedPathRecoveryState == SHARED_PATH_WAIT_GOT_IP) {
      telemetryHttpPauseRequested = true;
      delay(1);
      return;
    }

    if (WiFi.status() == WL_CONNECTED) {
      // v3.9.169: the ARP refresh experiment was disproved by v3.9.168 logs
      // and is deliberately not called. Normal lwIP ARP ownership is restored.
      // Blynk v0.6.1 may attempt a long server connection from inside
      // Blynk.run() when the configured server is not yet connected. Do not
      // call Blynk.run() while the server is completely unreachable. Once the
      // bounded TCP probe succeeds, however, keep calling Blynk.run() on every
      // loop pass until the existing handshake finishes. This is important: the
      // first Blynk.run() starts the handshake, but it may return before the
      // handshake is complete. The previous code waited for the next retry
      // interval, which is why your second handshake was always ~7-10 seconds
      // later. No second handshake is needed — the same Blynk session is simply
      // serviced again until it reaches Blynk.connected().
      unsigned long nowBlynk = millis();

#if CONNECTION_MODE == 0
      // v3.9.24: the probe is now a non-blocking state machine spread across
      // loop() passes (blynkProbeStart/blynkProbePoll) instead of a single
      // call that blocked Core 1 for up to BLYNK_SERVER_PROBE_TIMEOUT_MS.
      // blynkProbeInFlight tracks whether we are mid-probe; while true we
      // skip straight to polling it below instead of starting a new one.
      if (!blynkProbeInFlight &&
          blynkConfigured && !blynkHandshakeActive &&
          nowBlynk >= blynkStartupHoldoffUntil &&
          (blynkConnectImmediate ||
           nowBlynk - lastBlynkConnectAttempt >= BLYNK_CONNECT_RETRY_MS) &&
          (blynkForceReconnect || !Blynk.connected())) {
        // v3.9.37: consume the explicit immediate-connect request now that the
        // probe is actually being started. Subsequent attempts use the normal retry interval.
        blynkConnectImmediate = false;
        lastBlynkConnectAttempt = nowBlynk;

        if (blynkForceReconnect) {
          // Do this again immediately before the probe. The goal is not merely
          // to configure the server; it is to guarantee that a stale protocol
          // object/socket cannot suppress the new connection attempt.
          Blynk.disconnect();
#if CONNECTION_MODE == 0
          _stockBlynkTransportDiagnostics.disconnect();
#endif
          DBG_PRINTF("[Blynk] FORCED recovery probe — stale connected=%d cleared\n",
                     Blynk.connected() ? 1 : 0);
          // v3.9.40: give the new session a fresh RX-staleness window so it
          // isn't immediately re-flagged as stale before the new handshake
          // has had any chance to receive traffic.
          blynkLastRxMillis = millis();
        }

        // IMPORTANT: Blynk v0.6.1 Blynk.connect(timeout) is a blocking loop.
        // Even a nominal 1-second timeout can monopolize Core 1 long enough
        // to trigger the ESP32 Interrupt WDT when the TCP connection is not
        // ready. Probe the local Blynk TCP port first with a non-blocking
        // connect (see blynkProbeStart()/blynkProbePoll() above). Only start
        // Blynk.run() once the server has actually accepted a TCP
        // connection. This avoids the failure mode where an
        // unavailable/half-open Blynk server causes the CPU1 IWDT — and,
        // as of v3.9.24, without ever stalling Core 1 while polling.
        if (blynkProbeStart(BLYNK_SERVER, BLYNK_PORT)) {
          diagServerProbeStarts++;
          blynkProbeInFlight = true;
          // v3.9.24: unlike the old blocking probe, this no longer wraps
          // the whole multi-pass wait in one core1StageBegin/End pair —
          // that would incorrectly hold CORE1_STAGE_TCP_PROBE "open" across
          // many loop() passes while other stages (Blynk.run(), sensor
          // mutex service, etc.) actually execute, confusing the stall
          // monitor and V52/V53 diagnostics. Instead each poll below is
          // individually bracketed, so the recorded stage duration reflects
          // only the true (microsecond-scale) time actually spent inside
          // blynkProbePoll() on any single pass.
        } else {
          // Could not even start the probe (socket()/connect() failed
          // synchronously) — treat exactly like an immediate probe failure.
          if (blynkProbeFailCount < 255U) blynkProbeFailCount++;
          DBG_PRINTF("[Blynk] Server TCP probe failed to start — retry later (%u/%u)\n",
                        (unsigned)blynkProbeFailCount,
                        (unsigned)BLYNK_PROBE_FAILS_BEFORE_WIFI_RESET);
          if (blynkProbeFailCount >= BLYNK_PROBE_FAILS_BEFORE_WIFI_RESET) {
            DBG_PRINTLN("[Blynk] Repeated TCP probe failure — resetting Blynk/WiFi connection state");
            blynkProbeFailCount = 0;
            blynkHandshakeActive = false;
            blynkConfigured = false;
            Blynk.disconnect();
            wifiIntentionalTeardown("BLYNK_PROBE_START_FAILURE");
            wifiConnectInProgress = false;
            lastWiFiCheck = millis() - WIFI_RECONNECT_MS;
          }
        }
      }

      // Poll any in-flight probe. This costs microseconds per loop() pass —
      // select() below is called with a zero timeout — so Blynk.run() and
      // everything else in loop() keeps running normally while a probe is
      // pending, unlike the old blocking implementation.
      if (blynkProbeInFlight) {
        core1StageBegin(CORE1_STAGE_TCP_PROBE);
        bool probeDone = false;
        bool serverReachable = blynkProbePoll(BLYNK_SERVER_PROBE_TIMEOUT_MS, &probeDone);
        core1StageEnd();

        if (probeDone) {
          uint32_t probeElapsedMs = blynkProbe.startedAt ? (uint32_t)(millis() - blynkProbe.startedAt) : 0;
          if (probeElapsedMs > diagServerProbeMaxMs) diagServerProbeMaxMs = probeElapsedMs;
          blynkProbeInFlight = false;

          if (!serverReachable) {
            diagServerProbeFails++;
            if (blynkProbeFailCount < 255U) blynkProbeFailCount++;
            DBG_PRINTF("[Blynk] Server TCP probe failed — retry later (%u/%u)\n",
                          (unsigned)blynkProbeFailCount,
                          (unsigned)BLYNK_PROBE_FAILS_BEFORE_WIFI_RESET);

            // v3.9.85: a failed TCP probe is not proof that the associated
            // WiFi link is bad. If WiFi remains connected and no new unexpected
            // disconnect event occurred, recover Blynk only. This prevents the
            // firmware from creating a secondary WiFi status=6/reason=2 cascade.
            if (blynkProbeFailCount >= BLYNK_PROBE_FAILS_BEFORE_WIFI_RESET) {
              const bool wifiHealthyForProbeRecovery =
                  (WiFi.status() == WL_CONNECTED) &&
                  (wifiDisconnectEventSequence == blynkHandshakeWiFiDisconnectSeq);

              blynkProbeFailCount = 0;
              blynkHandshakeActive = false;
              Blynk.disconnect();
              blynkProbeAbort();
              blynkProbeInFlight = false;

              if (wifiHealthyForProbeRecovery) {
                blynkConfigured = true;
                blynkForceReconnect = false;
                blynkOfflineSince = millis();
                lastBlynkConnectAttempt = millis();
                blynkConnectImmediate = true;
                blynkLastRxMillis = millis();
                DBG_PRINTLN("[Blynk] Repeated TCP probe failure — BLYNK-ONLY recovery; WiFi untouched");
              } else {
                DBG_PRINTLN("[Blynk] Repeated TCP probe failure with unhealthy WiFi — WiFi recovery");
                blynkConfigured = false;
                wifiIntentionalTeardown("BLYNK_PROBE_FAILURE");
                wifiConnectInProgress = false;
                lastWiFiCheck = millis() - WIFI_RECONNECT_MS;
              }
            }
          } else {
            diagServerProbeSuccess++;
            blynkProbeFailCount = 0;
            // IMPORTANT:
            // Do NOT call Blynk.connect(timeout) here. Blynk v0.6.1 implements
            // connect() as a blocking loop and it has proved unreliable with this
            // local server even when the TCP port is reachable.
            //
            // The original firmware successfully established the Blynk session
            // through Blynk.run(). We preserve that behavior, but only allow
            // Blynk.run() to start the initial handshake after the bounded TCP
            // probe above has confirmed that the server is accepting connections.
            //
            // Once started, the handshake is NOT restarted every 10 seconds.
            // Blynk.run() is serviced on each loop pass until Blynk.connected()
            // becomes true. This is the normal way to advance the same Blynk
            // protocol session after its initial connection attempt.
            blynkHandshakeActive = true;
            blynkHandshakeStarted = millis();
            blynkHandshakeWiFiDisconnectSeq = wifiDisconnectEventSequence;
            DBG_PRINTLN("[Blynk] TCP port open — starting Blynk.run() handshake");
          }
        }
        // else: still waiting on this probe — Core 1 stays free to run
        // Blynk.run(), sensor mutex service, etc. below/above as normal.
      }

      if (blynkHandshakeActive && (blynkForceReconnect || !Blynk.connected())) {
        loopState = LOOP_STATE_BLYNK;
        // v3.3: Blynk v0.6.1 performs blocking socket reads inside Blynk.run().
        // The active raw transport in this branch has its own bounded socket
        // operations; BLYNK_TIMEOUT_MS remains the library-side read budget.
        // cannot monopolise Core 1 long enough to trip the ESP32 idle/interrupt
        // watchdog. If the server needs more time, the SAME Blynk session is
        // serviced again on the next loop pass — this is not a new handshake.
        // The short delay/yield below also gives the Arduino/ESP32 system tasks
        // a scheduling opportunity between successive protocol steps.
        {
          core1StageBegin(CORE1_STAGE_BLYNK_HANDSHAKE);
          uint32_t perfBlynkRunStartUs = micros();
          diagBlynkRunCalls++;
          Blynk.run();
          uint32_t blynkRunElapsedMs = (uint32_t)((micros() - perfBlynkRunStartUs) / 1000UL);
          perfRecordMax(perfMaxBlynkRunMs, blynkRunElapsedMs);
          if (blynkRunElapsedMs > BLYNK_RUN_GUARD_MS) diagBlynkRunSlow++;
          core1StageEnd();
          blynkRunGuardCheck(blynkRunElapsedMs, "handshake");
        }
        yield();

        if (Blynk.connected()) {
        loopState = LOOP_STATE_BLYNK;
          blynkHandshakeActive = false;
          blynkForceReconnect = false;
          blynkLastConnectedMillis = millis();
          blynkHandshakeFailCount = 0;
          blynkWifiRecoveryFailCount = 0;
          lastBlynkForcePing = millis();
          blynkForcePingCount = 0;
          // v3.9.43: see BLYNK_TX_POST_RECONNECT_GRACE_MS comment above —
          // hold off telemetry sends briefly after a fresh connection so a
          // backed-up queue does not immediately burst at a freshly-
          // recovering server.
          blynkTxGraceUntil = millis() + BLYNK_TX_POST_RECONNECT_GRACE_MS;
          if (blynkHandshakeStarted) {
            uint32_t handshakeMs = (uint32_t)(millis() - blynkHandshakeStarted);
            if (handshakeMs > diagBlynkHandshakeMaxMs) diagBlynkHandshakeMaxMs = handshakeMs;
          }
          DBG_PRINTF("[Blynk] Connected - Firmware %s\n", FIRMWARE_VERSION);
          engMsgf("Blynk connected - FW %s", FIRMWARE_VERSION);
        } else if (millis() - blynkHandshakeStarted >= BLYNK_HANDSHAKE_TIMEOUT_MS) {
          // The TCP probe succeeded, but the Blynk protocol handshake did not
          // finish in a bounded period. Stop servicing the failed session and
          // return to the normal retry state. This prevents an indefinitely
          // stuck connection attempt while still giving the working server
          // enough time to complete its handshake.
          blynkHandshakeActive = false;
          Blynk.disconnect();

          // v3.9.10: a successful TCP probe only proves that 192.168.3.4:8084
          // accepts TCP. The Blynk protocol still has to authenticate the
          // device. Do NOT tear down WiFi after the first login/handshake
          // timeout; doing that created the endless WiFi/Blynk reconnect loop
          // seen in v3.9.9. First and second failures only restart the Blynk
          // transport. After three consecutive handshake failures, reset the
          // WiFi association once. If three such WiFi recoveries fail, use the
          // existing safe software-reset path as a last-resort recovery.
          if (blynkHandshakeFailCount < 255U) blynkHandshakeFailCount++;
          blynkConfigured = true;
          blynkProbeFailCount = 0;
          lastBlynkConnectAttempt = millis();
          DBG_PRINTF("[Blynk] Handshake timeout — Blynk retry only (%u/%u)\n",
                        (unsigned)blynkHandshakeFailCount,
                        (unsigned)BLYNK_HANDSHAKE_FAILS_BEFORE_WIFI_RESET);

          if (blynkHandshakeFailCount >= BLYNK_HANDSHAKE_FAILS_BEFORE_WIFI_RESET) {
            blynkHandshakeFailCount = 0;

            // v3.9.84: IMPORTANT — a Blynk protocol failure must not be used
            // as proof that the ESP32 WiFi association is broken. The field
            // logs showed status=3, valid IP/GW, SO_ERROR=0 and healthy TX at
            // the exact moment the old code performed WiFi teardown #1. That
            // teardown then caused the secondary reason=2/status=6 reconnect
            // loop. If WiFi is still associated and no new unexpected WiFi
            // event occurred, reset ONLY the Blynk transport and leave WiFi
            // completely untouched.
            const bool wifiHealthyForBlynkRecovery =
                (WiFi.status() == WL_CONNECTED) &&
                (wifiDisconnectEventSequence == blynkHandshakeWiFiDisconnectSeq);

            blynkConfigured = true;
            Blynk.disconnect();
            blynkProbeAbort();
            blynkProbeInFlight = false;

            if (wifiHealthyForBlynkRecovery) {
              if (blynkWifiRecoveryFailCount < 255U) blynkWifiRecoveryFailCount++;
              blynkHandshakeActive = false;
              blynkForceReconnect = false;
              blynkOfflineSince = millis();
              lastBlynkConnectAttempt = millis();
              blynkConnectImmediate = true;
              blynkLastRxMillis = millis();
              DBG_PRINTF("[Blynk] Repeated handshake failure — BLYNK-ONLY transport reset (%u/%u); WiFi untouched status=%d RSSI=%d unexpected=%lu\n",
                         (unsigned)blynkWifiRecoveryFailCount,
                         (unsigned)BLYNK_WIFI_RECOVERY_CYCLE_THRESHOLD,
                         (int)WiFi.status(), WiFi.RSSI(),
                         (unsigned long)wifiUnexpectedDisconnectCount);

              // Do not count this as a WiFi recovery failure. The purpose of
              // this path is specifically to keep the WiFi association alive
              // while restarting only the failed Blynk session.
              if (blynkWifiRecoveryFailCount >= BLYNK_WIFI_RECOVERY_CYCLE_THRESHOLD) {
                // Repeated Blynk-only resets are still allowed to continue for
                // diagnostics; do not turn them into an automatic WiFi teardown.
                blynkWifiRecoveryFailCount = 0;
              }
            } else {
              // WiFi itself is no longer healthy, or an unexpected WiFi event
              // has already occurred. Only in this case is WiFi recovery
              // appropriate.
              if (blynkWifiRecoveryFailCount < 255U) blynkWifiRecoveryFailCount++;
              DBG_PRINTF("[Blynk] Handshake failure with unhealthy WiFi — WiFi/Blynk recovery (%u/%u) status=%d unexpected=%lu lastReason=%u\n",
                         (unsigned)blynkWifiRecoveryFailCount,
                         (unsigned)BLYNK_WIFI_RECOVERY_CYCLE_THRESHOLD,
                         (int)WiFi.status(),
                         (unsigned long)wifiUnexpectedDisconnectCount,
                         (unsigned)wifiLastDisconnectReason);
              blynkConfigured = false;
              wifiIntentionalTeardown("BLYNK_HANDSHAKE_FAILURE");
              wifiConnectInProgress = false;
              lastWiFiCheck = millis() - WIFI_RECONNECT_MS;

              if (blynkWifiRecoveryFailCount >= BLYNK_WIFI_RECOVERY_CYCLE_THRESHOLD) {
                DBG_PRINTLN("[Blynk] WiFi recovery limit reached - MCU preserved; continuing bounded recovery");
                // Communication recovery remains inside its own failure
                // domain. Continue bounded WiFi/Blynk retries without
                // restarting healthy sensor tasks.
                blynkWifiRecoveryFailCount = 0;
              }
            }
          }
        }
      }

      if (Blynk.connected()) {
        blynkOfflineSince = 0;
        blynkHandshakeFailCount = 0;
        blynkWifiRecoveryFailCount = 0;
        // Normal connected-state servicing. BLYNK_TIMEOUT_MS bounds any
        // waiting read if the server disappears between loop iterations.
        {
          core1StageBegin(CORE1_STAGE_BLYNK_RUN);
          uint32_t perfBlynkRunStartUs = micros();
          diagBlynkRunCalls++;
          Blynk.run();
          uint32_t blynkRunElapsedMs = (uint32_t)((micros() - perfBlynkRunStartUs) / 1000UL);
          perfRecordMax(perfMaxBlynkRunMs, blynkRunElapsedMs);
          if (blynkRunElapsedMs > BLYNK_RUN_GUARD_MS) diagBlynkRunSlow++;
          core1StageEnd();
          blynkRunGuardCheck(blynkRunElapsedMs, "connected");
        }
        yield();

        // v3.9.17: keep an explicit protocol-level heartbeat independent of
        // telemetry traffic. This is the key fix for the observed 63s
        // "Heartbeat timeout" while WiFi remained associated.
        // v3.9.92: do not inject an application-level PING every 10s.
        // Stock Blynk.run() owns the protocol heartbeat; an extra direct
        // sendCmd(PING) is another socket write and can hit the same ESP32
        // NetworkClient write-side stall that we are trying to avoid.

        core1StageBegin(CORE1_STAGE_BLYNK_TIMER);
        blynkTimer.run();
        core1StageEnd();

        // Transmit at most one queued telemetry packet per pass.  This is
        // deliberately after BlynkTimer so the timer callbacks only enqueue
        // data and never monopolise Core 1 with a large burst of writes.
        core1StageBegin(CORE1_STAGE_BLYNK_TX);
        blynkTxService();
        core1StageEnd();

        // v3.9.42 DIAGNOSTIC ONLY — see blynkStatusLogPassive() above. Purely
        // observational; does not affect connection state.
        blynkStatusLogPassive();
        // v3.9.66: passive RX-stall capture; diagnostic only, no recovery action.
        blynkRxStallDiagnosticPassive();
        blynkRxQueueTraceService();
        // v3.9.81: capture healthy->stalled transition independently of the
        // 10s stall diagnostic. Purely observational.
        blynkRxTraceService();
        // v3.9.80: recover a prolonged established-session RX stall without
        // tearing down a healthy WiFi association.
        blynkRxStallRecoveryService();

        // v3.9.9: Blynk can transition from connected to disconnected inside
        // Blynk.run(). Do not leave that condition waiting for the much longer
        // WiFi reconnect timer. Start a bounded recovery timer immediately.
        if (!Blynk.connected()) {
          if (blynkOfflineSince == 0) {
            blynkOfflineSince = millis();
            DBG_PRINTLN("[Blynk] Connection lost — starting recovery timer");

            // v3.9.11 FIX: this was the one recovery path in the whole state
            // machine that did NOT close the transport socket (the probe-fail,
            // handshake-timeout, and 60s-offline paths all call Blynk.disconnect()
            // + _boundedBlynkWifiClient.stop() before allowing a new handshake).
            // Left as-is, the top-level fast-retry block below sees
            // !Blynk.connected() && !blynkHandshakeActive within one loop pass
            // and immediately starts a brand-new handshake attempt on the SAME
            // WiFiClient object while it may still hold a half-closed/stale TCP
            // session from the connection that just died. That stale socket
            // cannot recover without a full teardown. The bounded recovery
            // timer below ensures the socket is closed and a brief settle period
            // elapses before a new handshake is attempted.
            Blynk.disconnect();
          }
        } else {
          // v3.9.33: Do not deliberately tear down a healthy Blynk connection
          // on a fixed 3-minute timer. A forced refresh hides the real failure
          // mechanism and creates an avoidable reconnect opportunity. Keep the
          // connection alive until Blynk reports an actual transition/failure.
          const unsigned long connectedAge = millis() - blynkLastConnectedMillis;
          if (connectedAge > 0 && (connectedAge % 60000UL) < 20UL) {
            DBG_PRINTF("[Blynk] Healthy connection age=%lums\n",
                       (unsigned long)connectedAge);
          }
        }
      }

#else  // CONNECTION_MODE == 1
  // v3.9.118: STOCK BLYNK RECOVERY — single session owner + bounded backoff.
  // Repeated protocol-login failures no longer cause an endless reconnect loop.
  // TCP reachability is logged separately from successful Blynk login.
  // v3.9.116: STOCK BLYNK RECOVERY — SINGLE SESSION OWNER + ACTIVE RECOVERY.
  // Blynk.run() remains the normal session owner. If the library reports a loss,
  // the sketch allows a 1s retry cadence and performs a single clean
  // Blynk.disconnect()+config refresh only after three failed retries.
  // WiFi is never torn down merely because Blynk heartbeat/session state fails.
  loopState = LOOP_STATE_BLYNK;
  unsigned long stockNow = millis();

  // A TX stall is an APPLICATION-TX fault, not a WiFi recovery event.  Keep
  // the stock protocol alive and suppress all virtualWrite/timer traffic
  // during the quarantine window.
  if (blynkStockTxRecoveryPending) {
    blynkStockEnterTxQuarantine(4, blynkTxLastCallMs);
    stockNow = millis();
  }

  // One-time Blynk.config() after WiFi becomes available.  There is no
  // Blynk.config() in the reconnect loop.
  if (!blynkConfigured && WiFi.status() == WL_CONNECTED) {
    Blynk.config(BLYNK_AUTH, BLYNK_SERVER, BLYNK_PORT);
    blynkConfigured = true;
    DBG_PRINTLN("[Blynk SIMPLE] config set — stock Blynk.run() owns connection");
  }

  // v3.9.162: make progress on bytes accepted by the Blynk transport while
  // lwIP was back-pressured. This never waits; TCP retransmission remains an
  // lwIP/WiFi responsibility and Blynk.run() is still called every loop pass.
  _blynkWifiClient.serviceDeferred();

  // v3.9.178: server-side capture proves this failure is an on-wire TCP
  // sequence hole.  lwIP accepted the affected write, so local EAGAIN,
  // deferred-byte and send-buffer counters all remain zero; they cannot be the
  // recovery gate.  A formerly healthy authenticated session with an unchanged
  // WiFi event sequence and no inbound Blynk response beyond the measured
  // response window is the end-to-end signal. v3.9.185 uses 25 seconds because
  // the supplied healthy maximum was 20.042s and the socket-only replacement
  // authenticated in about one second. The first action
  // is socket-only. v3.9.186 keeps all Blynk-only recovery socket-only and
  // admits no application frames until the replacement passes verification.
  const uint32_t blynkProtocolInboundAge =
      (uint32_t)(millis() - Blynk.lastInboundActivity());

  const bool blynkSessionWifiUnchanged =
      wifiDisconnectEventSequence == blynkSessionWiFiDisconnectSeq;
  const bool blynkApplicationProbeTimedOut =
      Blynk.applicationAckPending() &&
      Blynk.applicationAckAge() >= BLYNK_APPLICATION_PROBE_TIMEOUT_MS;
  const bool blynkProtocolProbeTimedOut =
      Blynk.protocolPingPending() &&
      Blynk.protocolPingAge() >= BLYNK_PROTOCOL_PROBE_TIMEOUT_MS;
  const bool blynkGenericInboundTimedOut =
      blynkProtocolInboundAge >= BLYNK_STOCK_PATH_RECOVERY_ESCALATE_MS;
  if (Blynk.connected() &&
      blynkStockSessionState == BLYNK_STOCK_SESSION_ONLINE &&
      blynkSessionWifiUnchanged &&
      _blynkWifiClient.connected() &&
      (blynkApplicationProbeTimedOut ||
       blynkProtocolProbeTimedOut ||
       blynkGenericInboundTimedOut)) {
    if (blynkSequenceHoleRecoveryTrial ||
        blynkSequenceHoleApplicationCanary) {
      Serial.printf("[Blynk TCP SEQUENCE HOLE] replacement session failed verification: phase=%s canary=%u/%u lastV=V%d inboundAge=%lums appProbe=%d/%lums protocolProbe=%d/%lums generic=%d sessionAge=%lums postLoginInbound=%d trialAge=%lums WiFi=%d RSSI=%d wifiDiscSeq=%lu unchanged=1 socketTrial=%u/%u\n",
                    blynkSequenceHoleApplicationCanary ? "APPLICATION_CANARY" : "HEARTBEAT_ONLY",
                    (unsigned)blynkApplicationCanaryPassed,
                    (unsigned)BLYNK_APPLICATION_CANARY_CHECKPOINTS,
                    (int)blynkTxLastVPin,
                    (unsigned long)blynkProtocolInboundAge,
                    Blynk.applicationAckPending() ? 1 : 0,
                    (unsigned long)Blynk.applicationAckAge(),
                    Blynk.protocolPingPending() ? 1 : 0,
                    (unsigned long)Blynk.protocolPingAge(),
                    blynkGenericInboundTimedOut ? 1 : 0,
                    (unsigned long)(millis() - blynkLastConnectedMillis),
                    blynkSessionSawPostLoginInbound ? 1 : 0,
                    (unsigned long)(millis() - blynkSequenceHoleRecoveryStartedMillis),
                    (int)WiFi.status(), WiFi.RSSI(),
                    (unsigned long)wifiDisconnectEventSequence,
                    (unsigned)blynkSequenceHoleSocketTrialCount,
                    (unsigned)BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS);
      blynkStockRxForensicSessionEnd(millis(), "tcp-sequence-hole-repeated");
      // This is not another trial of the original TCP sequence space. It is a
      // separately authenticated socket which received post-login traffic and
      // then became one-way during heartbeat/application verification. Count
      // that evidence immediately as generation two. v3.9.199 incorrectly
      // kept pathGenerations=1 here and rotated yet another socket, delaying
      // the recovery that its own evidence gate intended to authorize.
      if (blynkSequenceHoleRecentGenerations < 2U) {
        blynkSequenceHoleRecentGenerations = 2U;
      }
      const uint32_t repeatedHoleNow = millis();
      const uint32_t previousHoleAt = blynkSequenceHoleLastGenerationMillis;
      blynkSequenceHoleLastGenerationMillis = repeatedHoleNow;
      Serial.printf("[Blynk RX PATH GENERATION] count=%u source=replacement-session-one-way previousAge=%lums WiFiSeq=%lu\n",
                    (unsigned)blynkSequenceHoleRecentGenerations,
                    (unsigned long)(previousHoleAt ?
                        repeatedHoleNow - previousHoleAt : 0UL),
                    (unsigned long)wifiDisconnectEventSequence);
      blynkSequenceHoleRequestBoundedRestart("replacement-session-one-way");
      return;
    }

    Serial.printf("[Blynk TCP SEQUENCE HOLE] established stream probe failed: inboundAge=%lums appProbe=%d id=%u age=%lums protocolProbe=%d id=%u age=%lums generic=%d; tcpConnected=1 pending=%u EAGAIN=%lu socketErr=%lu WiFi=%d RSSI=%d wifiDiscSeq=%lu unchanged=1; rotating Blynk socket first\n",
                  (unsigned long)blynkProtocolInboundAge,
                  Blynk.applicationAckPending() ? 1 : 0,
                  (unsigned)Blynk.applicationAckPingId(),
                  (unsigned long)Blynk.applicationAckAge(),
                  Blynk.protocolPingPending() ? 1 : 0,
                  (unsigned)Blynk.protocolPingId(),
                  (unsigned long)Blynk.protocolPingAge(),
                  blynkGenericInboundTimedOut ? 1 : 0,
                  (unsigned)_blynkWifiClient.pendingBytes(),
                  (unsigned long)blynkTransportWriteEagain,
                  (unsigned long)blynkTransportSocketErrors,
                  (int)WiFi.status(), WiFi.RSSI(),
                  (unsigned long)wifiDisconnectEventSequence);
    blynkStockRxForensicSessionEnd(millis(), "tcp-sequence-hole");

    // v3.9.211: one failed stream can be repaired by rotating that stream. It
    // is the wrong reset domain once the independently owned HTTP stream has
    // also timed out recently. The v3.9.210 field trace reached this exact
    // state (HTTP code -11, then exact Blynk PING loss); socket rotation then
    // produced a login-only one-way replacement and merely delayed the MCU
    // reset which finally restored traffic.
    const uint32_t sequenceHoleNow = millis();
    const bool recentHttpTransportTimeout =
        telemetryHttpLastFailureMs != 0 &&
        sequenceHoleNow - telemetryHttpLastFailureMs <=
            BLYNK_SHARED_PATH_CORROBORATION_MS;
    // v3.9.212: telemetryHttpLastFailureMs intentionally remains available for
    // diagnostics after HTTP recovers. It must therefore not, by itself, prove
    // that the second path is still broken. Require an unresolved failure
    // streak; a recovered one-off timeout receives socket-first Blynk repair.
    const bool unresolvedRecentHttpFailure =
        recentHttpTransportTimeout &&
        telemetryHttpConsecutiveFailures >= 2U;
    const bool activeHttpCircuit =
        telemetryHttpCircuitOpenUntilMs != 0 &&
        (int32_t)(telemetryHttpCircuitOpenUntilMs - sequenceHoleNow) > 0;
    // A circuit is a pause timer, not fresh failure evidence. It deliberately
    // remains open for 90 seconds, so using it alone made a single recovered
    // timeout force an unrelated Blynk hole directly into esp_restart().
    if (unresolvedRecentHttpFailure) {
      blynkSequenceHoleSocketTrialCount = 0;
      blynkSequenceHoleRecentGenerations = 2U;
      const uint32_t httpFailureAge = telemetryHttpLastFailureMs
          ? sequenceHoleNow - telemetryHttpLastFailureMs : 0UL;
      blynkSequenceHoleLastGenerationMillis = sequenceHoleNow;
      Serial.printf("[Blynk SHARED PATH PROOF] exact Blynk sequence hole plus unresolved HTTP evidence: httpRecent=%d consecutive=%u httpAge=%lums circuit=%d httpFailures=%lu; bypassing ineffective Blynk-only reset\n",
                    recentHttpTransportTimeout ? 1 : 0,
                    (unsigned)telemetryHttpConsecutiveFailures,
                    (unsigned long)httpFailureAge,
                    activeHttpCircuit ? 1 : 0,
                    (unsigned long)telemetryHttpFailureCount);
      blynkSequenceHoleRequestBoundedRestart(
          "blynk-hole-plus-http-transport-failure");
      return;
    }

    if (recentHttpTransportTimeout && !unresolvedRecentHttpFailure) {
      Serial.printf("[Blynk SHARED PATH NOT PROVEN] recent HTTP failure recovered/isolated (consecutive=%u age=%lums); trying Blynk socket-only repair first\n",
                    (unsigned)telemetryHttpConsecutiveFailures,
                    (unsigned long)(sequenceHoleNow - telemetryHttpLastFailureMs));
    }

    // This is a genuinely new failure generation only when neither heartbeat
    // nor application canary verification is active. With no corroborating
    // HTTP failure, preserve the narrow Blynk-socket-first recovery.
    blynkSequenceHoleSocketTrialCount = 0;
    if (blynkSequenceHoleLastGenerationMillis != 0 &&
        sequenceHoleNow - blynkSequenceHoleLastGenerationMillis <=
            BLYNK_SEQUENCE_HOLE_RECURRENCE_WINDOW_MS) {
      if (blynkSequenceHoleRecentGenerations < 255U) {
        ++blynkSequenceHoleRecentGenerations;
      }
    } else {
      blynkSequenceHoleRecentGenerations = 1U;
    }
    const uint32_t previousGenerationAt =
        blynkSequenceHoleLastGenerationMillis;
    blynkSequenceHoleLastGenerationMillis = sequenceHoleNow;
    Serial.printf("[Blynk RX PATH GENERATION] count=%u window=%lums previousAge=%lums WiFiSeq=%lu\n",
                  (unsigned)blynkSequenceHoleRecentGenerations,
                  (unsigned long)BLYNK_SEQUENCE_HOLE_RECURRENCE_WINDOW_MS,
                  (unsigned long)(previousGenerationAt ?
                      sequenceHoleNow - previousGenerationAt : 0UL),
                  (unsigned long)wifiDisconnectEventSequence);

    // v3.9.202: both ESP-IDF 5.5.5 and 6.1 proved that touching STA here can
    // turn a bounded Blynk outage into a long association failure. A fresh TCP
    // sequence space is the narrow first repair. The recovery-trial predicate
    // above escalates only if this independently authenticated socket also
    // becomes one-way.
    blynkSequenceHoleRotateSocket("first-proven-on-wire-sequence-hole");
    return;
  }

  // If the stock object is disconnected, simply service it at a bounded
  // cadence. Normal retries remain Blynk-only. After a sustained failure
  // following a real online session, one bounded escalation may rebuild the
  // WiFi/lwIP interface because WL_CONNECTED can survive a poisoned TCP
  // transmit path (confirmed by the v3.9.147 field log).
  if (!Blynk.connected()) {
    if (blynkStockSessionState == BLYNK_STOCK_SESSION_ONLINE) {
      blynkStockRxForensicSessionEnd(stockNow, "stock-object-disconnected");
      blynkRecordSessionLossAndMaybeRestart("stock-object-disconnected");
      blynkPurgeClosedSessionPcbs("stock-object-disconnected");
      DBG_PRINTLN("[Blynk SIMPLE] session disconnected — returning to stock retry");
      blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
      blynkStockStateSinceMillis = stockNow;
      blynkOfflineSince = stockNow;
      blynkStockNextRunMillis = stockNow +
          BLYNK_STOCK_RETRY_INTERVAL_MS;
      blynkTxDropCoherentPair();
      blynkStockTxRecoveryPending = false;
      DBG_PRINTF("[Blynk FORENSIC] session loss observed; reconnect hold=%lums\n",
                 (unsigned long)BLYNK_STOCK_RETRY_INTERVAL_MS);
      if (blynkSequenceHoleRecoveryTrial ||
          blynkSequenceHoleApplicationCanary) {
        if (blynkSequenceHoleSocketTrialCount <
            BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS) {
          blynkSequenceHoleRotateSocket("verification-session-disconnected");
        } else {
          blynkSequenceHoleEnterServerCooldown(
              "three-verification-session-disconnects");
        }
        return;
      }
    }

    // v3.9.161: no sidecar/preflight socket. The actual Blynk WiFiClient is
    // the only connection authority and its connect() is bounded to 1000ms.
    // Any stale probe left by an older state is closed once, then normal stock
    // Blynk.run() reconnects the real protocol session while WiFi stays intact.
    if (blynkProbeInFlight) blynkProbeAbort();
    blynkProbeInFlight = false;
    blynkStockTcpPreflightReady = false;
    blynkStockTcpPreflightFailures = 0;
    const bool blynkStockTcpGateOpen = true;

    if (blynkStockTcpGateOpen &&
        WiFi.status() == WL_CONNECTED &&
        (blynkStockNextRunMillis == 0 ||
         (int32_t)(stockNow - blynkStockNextRunMillis) >= 0)) {
      const unsigned long retryStart = millis();
      static unsigned long blynkLoginWaitSince = 0;  // v3.9.141
      // The real stock transport performs this bounded connection attempt.
      blynkStockTcpPreflightReady = false;
      blynkStockSessionState = BLYNK_STOCK_SESSION_CONNECTING;
      blynkStockStateSinceMillis = retryStart;
      lastBlynkConnectAttempt = retryStart;
      if (blynkLoginWaitSince == 0) ++blynkStockReconnectCount;
      if (blynkLoginWaitSince == 0) DBG_PRINTF("[Blynk SIMPLE] stock retry #%lu via Blynk.run() WiFi=OK RSSI=%d cleanRefreshes=%u\n",
                 (unsigned long)blynkStockReconnectCount, WiFi.RSSI(),
                 (unsigned)blynkStockCleanRefreshCount);

      const unsigned long stockRunRxAgeBefore = blynkStockRxProxyAgeMs();
      _stockBlynkTransportDiagnostics.diagnosticResponseHeaderPeek("before-stock-retry");
      core1StageBegin(CORE1_STAGE_BLYNK_RUN);
      // v3.9.122 DIAGNOSTIC: every individual transport operation
      // (connect/read/write/connected/available) is independently bounded
      // to well under 3000ms, yet this call site measures ~3000ms
      // end-to-end around Blynk.run(). That combination is only possible
      // if either (a) some untimed operation inside run() is not actually
      // bounded the way it's assumed to be, or (b) the Core-1 loop() task
      // is being preempted by the FreeRTOS scheduler for ~3s while wall-
      // clock time keeps advancing, which this wrapper cannot distinguish
      // from real execution time. Splitting the measurement around the
      // yield boundary at BLYNK_RUN_YIELD() (the very first line of
      // run()) will show which side of that boundary the time is lost on:
      // if the gap appears before any library code runs, it's scheduler
      // preemption, not a transport stall.
      uint32_t perfPreYieldUs = micros();
      BLYNK_RUN_YIELD();
      uint32_t perfPostYieldUs = micros();
      uint32_t perfBlynkRunStartUs = micros();
      diagBlynkRunCalls++;
      Blynk.run();
      uint32_t perfBlynkRunEndUs = micros();
      uint32_t blynkRunElapsedMs =
          (uint32_t)((perfBlynkRunEndUs - perfBlynkRunStartUs) / 1000UL);
      uint32_t yieldGapMs = (uint32_t)((perfPostYieldUs - perfPreYieldUs) / 1000UL);
      if (blynkRunElapsedMs > BLYNK_RUN_GUARD_MS || yieldGapMs > 50UL) {
        Serial.printf("[Blynk RUN SPLIT] phase=stock-retry yieldGapMs=%lums runBodyMs=%lums (yield is BLYNK_RUN_YIELD() at the top of stock run() — a large yieldGapMs with a small runBodyMs means the Core-1 task was descheduled BEFORE any library code executed, not stalled inside a transport call)\n",
                      (unsigned long)yieldGapMs, (unsigned long)blynkRunElapsedMs);
      }
      blynkStockRecordRunForensic(blynkRunElapsedMs, stockRunRxAgeBefore, "stock-retry");
      _stockBlynkTransportDiagnostics.diagnosticResponseHeaderPeek("after-stock-retry");
      perfRecordMax(perfMaxBlynkRunMs, blynkRunElapsedMs);
      if (blynkRunElapsedMs > BLYNK_RUN_GUARD_MS) diagBlynkRunSlow++;
      core1StageEnd();
      blynkRunGuardCheck(blynkRunElapsedMs, "stock-retry");
      yield();

      if (Blynk.connected()) {
        blynkLoginWaitSince = 0;
        blynkStockConsecutiveRetryFailures = 0;
        blynkStockOutageTcpConnectFailures = 0;
        blynkStockOutageConnectFailureBaseline = blynkTransportConnectFailures;
        blynkStockOutageTcpConnectTimeouts = 0;
        blynkStockOutageConnectTimeoutBaseline = blynkTransportConnectTimeouts;
        // A login inside an unresolved recovery generation is not a stable
        // recovery. Preserve outage backoff until application canaries pass.
        if (!blynkSequenceHoleRecoveryTrial &&
            !blynkSequenceHoleApplicationCanary) {
          blynkStockCleanRefreshCount = 0;
        }
        blynkStockWifiStackRecoveryAttempted = false;
        blynkStockWifiStackRecoveryInProgress = false;
        blynkStockTcpPreflightReady = false;
        blynkStockTcpPreflightFailures = 0;
        const unsigned long connectedAt = millis();
        blynkStockRxForensicSessionStart(connectedAt);
        blynkStockLastTxProbeOkMillis = 0;
        blynkStockNextTxProbeMillis = connectedAt;
        blynkStockTxProbeFailures = 0;
        blynkStockTxProbeInFlight = false;
        blynkStockSessionState = BLYNK_STOCK_SESSION_ONLINE;
        blynkStockStateSinceMillis = connectedAt;
        blynkOfflineSince = 0;
        blynkHandshakeActive = false;
        blynkHandshakeFailCount = 0;
        blynkWifiRecoveryFailCount = 0;
        blynkStockNextRunMillis = 0;
        blynkStockTxRecoveryPending = false;
        blynkStockConsecutiveRetryFailures = 0;
        blynkStockTxQuarantineUntil = connectedAt + BLYNK_STOCK_POST_RECONNECT_STABLE_MS;
        blynkStockStableUntilMillis = blynkStockTxQuarantineUntil;
        blynkTxGraceUntil = blynkStockTxQuarantineUntil;
        blynkLastConnectedMillis = connectedAt;
        blynkSessionInboundBaseline = Blynk.lastInboundActivity();
        blynkSessionWiFiDisconnectSeq = wifiDisconnectEventSequence;
        blynkSessionSawPostLoginInbound = false;
        blynkLastTxSuccessMillis = connectedAt;
        blynkLastRxMillis = connectedAt;
        lastBlynkForcePing = connectedAt;
        blynkForcePingCount = 0;
        DBG_PRINTLN("[Blynk SIMPLE] CONNECTED — stock Blynk.run() owns heartbeat/session; post-connect TX quarantine active");
        engMsgf("Blynk SIMPLE connected - FW %s", FIRMWARE_VERSION);
      } else if (_blynkWifiClient.connected() &&
                 (blynkLoginWaitSince == 0 || millis() - blynkLoginWaitSince < 5000UL)) {
        // v3.9.141: TCP is up and HW_LOGIN was sent, but stock Blynk.run()
        // returns before the server's RESPONSE is read. The library only waits
        // BLYNK_TIMEOUT_MS for it, so run() must be called again promptly
        // instead of after a 5-60s retry backoff (which guaranteed a
        // "Login timeout" / reconnect loop).
        if (blynkLoginWaitSince == 0) blynkLoginWaitSince = millis();
        blynkStockSessionState = BLYNK_STOCK_SESSION_CONNECTING;
        blynkStockNextRunMillis = millis() + 20UL;
      } else {
        blynkLoginWaitSince = 0;
        blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
        if (blynkOfflineSince == 0) blynkOfflineSince = millis();
        if (blynkStockConsecutiveRetryFailures < 0xFFFFFFFFUL) {
          ++blynkStockConsecutiveRetryFailures;
        }

        // Derive the outage count from the transport's real connect() failure
        // counter. This cannot count Blynk.run() passes which merely change
        // protocol state without opening a socket (the v3.9.170 error).
        const uint32_t actualConnectFailures = blynkTransportConnectFailures;
        if (actualConnectFailures >= blynkStockOutageConnectFailureBaseline) {
          const uint32_t outageFailures =
              actualConnectFailures - blynkStockOutageConnectFailureBaseline;
          blynkStockOutageTcpConnectFailures =
              (uint8_t)(outageFailures > 255UL ? 255UL : outageFailures);
        } else {
          // Defensive handling of the 32-bit diagnostic counter wrapping.
          blynkStockOutageConnectFailureBaseline = actualConnectFailures;
          blynkStockOutageTcpConnectFailures = 0;
        }

        const uint32_t actualConnectTimeouts = blynkTransportConnectTimeouts;
        if (actualConnectTimeouts >= blynkStockOutageConnectTimeoutBaseline) {
          const uint32_t outageTimeouts =
              actualConnectTimeouts - blynkStockOutageConnectTimeoutBaseline;
          blynkStockOutageTcpConnectTimeouts =
              (uint8_t)(outageTimeouts > 255UL ? 255UL : outageTimeouts);
        } else {
          blynkStockOutageConnectTimeoutBaseline = actualConnectTimeouts;
          blynkStockOutageTcpConnectTimeouts = 0;
        }

        // Ordinary refusals/login failures still remain Blynk-only. Escalate
        // exactly once only when this outage follows a proven sequence hole
        // and three fresh TCP attempts each receive no result for the entire
        // one-second budget. That is the field-observed false-connected path.
        bool requestedPathRestart = false;
        if (blynkSequenceHoleRecentGenerations > 0U &&
            !blynkStockWifiStackRecoveryAttempted &&
            blynkStockOutageTcpConnectTimeouts >=
                BLYNK_STOCK_RX_PATH_TIMEOUTS_BEFORE_RESTART) {
          blynkSequenceHoleRequestBoundedRestart("two-post-hole-tcp-timeouts");
          requestedPathRestart = true;
        }

        // Exact v0.6.1 behavior: Blynk.run() can leave the protocol object in
        // CONNECTING after a failed login/connection attempt. Repeating run()
        // is normally sufficient, but after several failures we deliberately
        // reset ONLY the Blynk protocol state. This does not touch WiFi and
        // avoids building a second, sketch-owned connection state machine.
        // v3.9.118: a TCP-open server can still reject or fail the Blynk
        // protocol handshake. Do not hammer it every 5 seconds forever.
        // Backoff is measured from the END of Blynk.run(), so a 2.8-3.0s
        // stock connect timeout cannot immediately retrigger another attempt.
        if (requestedPathRestart) {
          DBG_PRINTLN("[Blynk RX PATH RECOVERY] controlled restart requested");
        } else if (blynkStockCleanRefreshCount == 0U &&
            blynkStockConsecutiveRetryFailures >= BLYNK_STOCK_CLEAN_REFRESH_AFTER_RETRIES &&
            (blynkStockLastCleanRefreshMillis == 0 ||
             millis() - blynkStockLastCleanRefreshMillis >= BLYNK_STOCK_CLEAN_REFRESH_MIN_MS)) {
          const unsigned long refreshAt = millis();
          if (blynkStockCleanRefreshCount < 255U) ++blynkStockCleanRefreshCount;

          // After several complete clean-refresh cycles with no successful
          // Blynk login, stop probing the same dead/invalid protocol endpoint
          // for minutes. This is not a WiFi failure, so WiFi stays associated.
          const bool longHold =
              blynkStockCleanRefreshCount >= BLYNK_STOCK_REFRESHES_BEFORE_LONG_HOLDOFF;
          const unsigned long holdoff = longHold ?
              BLYNK_STOCK_LONG_HOLDOFF_MS : BLYNK_STOCK_REFRESH_HOLDOFF_MS;

          DBG_PRINTF("[Blynk RECOVERY] %lu failed stock retries — clean Blynk session refresh #%u; TCP may be reachable but Blynk service/login is not completing; WiFi untouched\n",
                     (unsigned long)blynkStockConsecutiveRetryFailures,
                     (unsigned)blynkStockCleanRefreshCount);
          blynkStockProtocolReset();
          blynkStockLastCleanRefreshMillis = refreshAt;
          blynkStockConsecutiveRetryFailures = 0;
          blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
          blynkStockStateSinceMillis = refreshAt;
          blynkOfflineSince = refreshAt;
          blynkStockNextRunMillis = refreshAt + holdoff;
          blynkStockTxRecoveryPending = false;
          blynkTxDropCoherentPair();
          blynkStockLastTxProbeOkMillis = 0;
          blynkStockTxProbeInFlight = false;
          if (blynkProbeInFlight) blynkProbeAbort();
          DBG_PRINTF("[Blynk RECOVERY] clean refresh complete; next stock Blynk.run in %lums (%s holdoff)\n",
                     (unsigned long)holdoff, longHold ? "LONG" : "NORMAL");
        } else if (blynkStockCleanRefreshCount > 0U &&
                   blynkStockConsecutiveRetryFailures >= BLYNK_STOCK_CLEAN_REFRESH_AFTER_RETRIES) {
          // One protocol reset is sufficient to clear stale Blynk 0.6.1 state.
          // Further resets cannot repair an unavailable server or lower path;
          // preserve state and reduce connection/restart churn instead.
          const unsigned long holdAt = millis();
          DBG_PRINTF("[Blynk RECOVERY] one clean refresh already used; preserving protocol and entering %lums quiet holdoff after %lu further failures\n",
                     (unsigned long)BLYNK_STOCK_LONG_HOLDOFF_MS,
                     (unsigned long)blynkStockConsecutiveRetryFailures);
          blynkStockConsecutiveRetryFailures = 0;
          blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
          blynkStockStateSinceMillis = holdAt;
          blynkOfflineSince = holdAt;
          blynkStockNextRunMillis = holdAt + BLYNK_STOCK_LONG_HOLDOFF_MS;
          blynkStockTxRecoveryPending = false;
          blynkTxDropCoherentPair();
        } else {
          // First failed login waits the base 1s, then 2s, 4s and at most 8s.
          // The old expression started at 10s because it shifted the 5s base
          // immediately on failure #1, compounding even brief LAN glitches.
          uint32_t shift = blynkStockConsecutiveRetryFailures > 0 ?
              blynkStockConsecutiveRetryFailures - 1U : 0U;
          if (shift > 3U) shift = 3U;
          unsigned long retryDelay = BLYNK_STOCK_RETRY_INTERVAL_MS << shift;
          if (retryDelay > BLYNK_STOCK_RETRY_MAX_MS) retryDelay = BLYNK_STOCK_RETRY_MAX_MS;
          blynkStockNextRunMillis = millis() + retryDelay;
          DBG_PRINTF("[Blynk SIMPLE] stock retry ended disconnected after %lums; failedRetries=%lu next retry in %lums\n",
                     (unsigned long)blynkRunElapsedMs,
                     (unsigned long)blynkStockConsecutiveRetryFailures,
                     (unsigned long)retryDelay);
        }
      }
    }
  } else {
    // CONNECTED: Blynk.run() is always serviced, including TX quarantine.
    // The application timer/TX layer is deliberately suspended until the
    // post-reconnect stability window has elapsed.
    const unsigned long stockRunRxAgeBefore = blynkStockRxProxyAgeMs();
    const bool stockTcpConnectedBeforeRun = _blynkWifiClient.connected();
    _stockBlynkTransportDiagnostics.diagnosticResponseHeaderPeek("before-Blynk.run");
    core1StageBegin(CORE1_STAGE_BLYNK_RUN);
    uint32_t perfBlynkRunStartUs = micros();
    diagBlynkRunCalls++;
    Blynk.run();
    uint32_t blynkRunElapsedMs =
        (uint32_t)((micros() - perfBlynkRunStartUs) / 1000UL);
    blynkStockRecordRunForensic(blynkRunElapsedMs, stockRunRxAgeBefore, "connected");
    _stockBlynkTransportDiagnostics.diagnosticResponseHeaderPeek("after-Blynk.run");
    perfRecordMax(perfMaxBlynkRunMs, blynkRunElapsedMs);
    if (blynkRunElapsedMs > BLYNK_RUN_GUARD_MS) diagBlynkRunSlow++;
    core1StageEnd();
    blynkRunGuardCheck(blynkRunElapsedMs, "connected");
    blynkStockRxForensicService("after-Blynk.run");
    yield();

    const bool blynkConnectedAfterRun = Blynk.connected();
    if (blynkConnectedAfterRun) {
      const uint32_t inboundNow = Blynk.lastInboundActivity();
      if (!blynkSessionSawPostLoginInbound &&
          inboundNow != blynkSessionInboundBaseline) {
        blynkSessionSawPostLoginInbound = true;
        blynkConsecutiveLoginOnlyLosses = 0;
        Serial.printf("[Blynk SESSION] post-login inbound confirmed after %lums; login-only stall streak cleared\n",
                      (unsigned long)(millis() - blynkLastConnectedMillis));
      }
      // Do not clear the recovery trial on a single immediate post-login frame:
      // the failing v3.9.178 session received one after 20ms and then developed
      // another sequence hole. Require a stable interval plus currently fresh
      // inbound traffic (normally a response to the 10-second protocol PING).
      if (blynkSequenceHoleRecoveryTrial &&
          millis() - blynkLastConnectedMillis >= BLYNK_STOCK_PATH_RECOVERY_VERIFY_MS &&
          (uint32_t)(millis() - inboundNow) < BLYNK_STOCK_APPLICATION_TX_PAUSE_RX_MS) {
        Serial.printf("[Blynk TCP SEQUENCE HOLE] heartbeat-only trial %u verified after %lums; entering %u-checkpoint application canary, full publishers remain gated\n",
                      (unsigned)blynkSequenceHoleSocketTrialCount,
                      (unsigned long)(millis() - blynkLastConnectedMillis),
                      (unsigned)BLYNK_APPLICATION_CANARY_CHECKPOINTS);
        blynkSequenceHoleRecoveryTrial = false;
        blynkSequenceHoleApplicationCanary = true;
        blynkApplicationCanaryPassed = 0;
        blynkApplicationCanarySentBaseline =
            Blynk.applicationAckSentCount();
        blynkApplicationCanaryVerifiedBaseline =
            Blynk.applicationAckVerifiedCount();
      }
      if (blynkSequenceHoleApplicationCanary &&
          !Blynk.applicationAckPending() &&
          Blynk.applicationAckSentCount() >
              blynkApplicationCanarySentBaseline &&
          Blynk.applicationAckVerifiedCount() >
              blynkApplicationCanaryVerifiedBaseline) {
        if (blynkApplicationCanaryPassed < 255U) {
          ++blynkApplicationCanaryPassed;
        }
        blynkApplicationCanarySentBaseline =
            Blynk.applicationAckSentCount();
        blynkApplicationCanaryVerifiedBaseline =
            Blynk.applicationAckVerifiedCount();
        Serial.printf("[Blynk APPLICATION CANARY] checkpoint %u/%u verified lastV=V%d frames=%lu verified=%lu\n",
                      (unsigned)blynkApplicationCanaryPassed,
                      (unsigned)BLYNK_APPLICATION_CANARY_CHECKPOINTS,
                      (int)blynkTxLastVPin,
                      (unsigned long)Blynk.applicationAckSentCount(),
                      (unsigned long)Blynk.applicationAckVerifiedCount());
        if (blynkApplicationCanaryPassed >=
            BLYNK_APPLICATION_CANARY_CHECKPOINTS) {
          Serial.printf("[Blynk APPLICATION CANARY] all %u checkpoints verified; normal publishers released, recovery generation complete\n",
                        (unsigned)BLYNK_APPLICATION_CANARY_CHECKPOINTS);
          blynkSequenceHoleApplicationCanary = false;
          blynkApplicationCanaryPassed = 0;
          blynkSequenceHoleRecoveryStartedMillis = 0;
          blynkSequenceHoleSocketTrialCount = 0;
          blynkStockCleanRefreshCount = 0;
        }
      }
      // Keep protocol liveness independent of telemetry. Because application
      // writes count as lastActivityOut in Blynk 0.6.1, the library's normal
      // heartbeat condition can otherwise remain suppressed indefinitely.
      // v3.9.147: no sidecar TCP probe. It exercised a different socket and
      // could not establish whether the live Blynk connection was healthy.
      // v3.9.167: the patched BlynkProtocol is the single heartbeat owner.
      // Running the old sketch heartbeat as well would enqueue duplicate PINGs
      // behind a missing TCP segment and amplify the exact failure under test.
      // v3.9.132: check for an armed-but-never-answered PING watch every
      // loop pass. Use the full heartbeat deadline as the timeout so this
      // fires at the same moment the library's own heartbeat-timeout logic
      // would otherwise give up — this is what tells us definitively
      // whether that disconnect corresponds to an unanswered PING (this
      // watch times out) or arrives with no armed watch at all.
      blynkPingRttWatchCheckTimeout(millis(), BLYNK_HEARTBEAT * 1000UL + BLYNK_TIMEOUT_MS * 3UL);
    }

    if (!Blynk.connected()) {
      // heartbeat-sendcmd already records and transitions the session itself.
      // Handle only a still-ONLINE loss here to avoid duplicate session-end
      // accounting and misleading "Blynk.run ended" attribution.
      if (blynkStockSessionState == BLYNK_STOCK_SESSION_ONLINE) {
        const char* lossSource = blynkConnectedAfterRun ?
            "heartbeat-send" : "Blynk.run";
        Serial.printf("[Blynk LOSS] source=%s tcpBeforeRun=%d tcpAfter=%d runMs=%lums sessionAge=%lums rxObservedAge=%lums WiFi=%d RSSI=%d wifiDiscEvents=%lu\n",
                      lossSource,
                      stockTcpConnectedBeforeRun ? 1 : 0,
                      _blynkWifiClient.connected() ? 1 : 0,
                      (unsigned long)blynkRunElapsedMs,
                      (unsigned long)(millis() - blynkLastConnectedMillis),
                      (unsigned long)blynkStockRxProxyAgeMs(),
                      (int)WiFi.status(), WiFi.RSSI(),
                      (unsigned long)wifiUnexpectedDisconnectCount);
        blynkStockRxForensicSessionEnd(millis(), lossSource);
        blynkRecordSessionLossAndMaybeRestart(lossSource);
      }
      if (blynkSequenceHoleRecoveryTrial ||
          blynkSequenceHoleApplicationCanary) {
        if (blynkSequenceHoleSocketTrialCount <
            BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS) {
          blynkSequenceHoleRotateSocket(
              "verification-session-run-disconnect");
        } else {
          blynkSequenceHoleEnterServerCooldown(
              "three-verification-run-disconnects");
        }
        return;
      }
      // BlynkProtocol has already closed the socket through its owner. The
      // compatibility helper deliberately performs no raw lwIP PCB mutation.
      blynkPurgeClosedSessionPcbs("run-session-loss");
      blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
      blynkStockStateSinceMillis = millis();
      blynkOfflineSince = millis();
      blynkStockNextRunMillis = millis() + BLYNK_STOCK_RETRY_INTERVAL_MS;
      blynkStockTxRecoveryPending = false;
      blynkTxDropCoherentPair();
    } else if (millis() < blynkStockTxQuarantineUntil) {
      // Circuit breaker: no application TX while stock Blynk settles.
    } else if (blynkSequenceHoleRecoveryTrial) {
      // v3.9.186: genuine protocol-only probation. Do not even run timer
      // producers, so no stale queue is accumulated for release at 30 seconds.
      // Blynk.run() above remains fully active and owns heartbeat verification.
    } else {
      // Normal publishers only enqueue values; the paced TX service below
      // sends at most one packet per loop pass. During an application canary,
      // blynkStockApplicationTxAllowed() admits only one exactly checkpointed
      // frame at a time until all canary checkpoints pass.
      blynkTimer.run();
      if (Blynk.connected()) {
        blynkTxService();
      }
      if (!Blynk.connected()) {
        Serial.printf("[Blynk LOSS] source=publisher lastV=V%d writeMs=%lums sessionAge=%lums WiFi=%d RSSI=%d wifiDiscEvents=%lu\n",
                      (int)blynkTxLastVPin, (unsigned long)blynkTxLastCallMs,
                      (unsigned long)(millis() - blynkLastConnectedMillis),
                      (int)WiFi.status(), WiFi.RSSI(),
                      (unsigned long)wifiUnexpectedDisconnectCount);
        blynkStockRxForensicSessionEnd(millis(), "publisher-send");
        blynkRecordSessionLossAndMaybeRestart("publisher-send");
        if (blynkSequenceHoleApplicationCanary) {
          if (blynkSequenceHoleSocketTrialCount <
              BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS) {
            blynkSequenceHoleRotateSocket("application-canary-send-disconnect");
          } else {
            blynkSequenceHoleEnterServerCooldown(
                "three-application-canary-send-disconnects");
          }
          return;
        }
        blynkPurgeClosedSessionPcbs("publisher-session-loss");
        blynkStockSessionState = BLYNK_STOCK_SESSION_IDLE;
        blynkStockStateSinceMillis = millis();
        blynkOfflineSince = millis();
        blynkStockNextRunMillis = millis() + BLYNK_STOCK_RETRY_INTERVAL_MS;
        blynkTxDropCoherentPair();
      }
    }
  }
#endif  // CONNECTION_MODE



    }
  }

  // Evaluate health only after Blynk.run() had the opportunity to consume an
  // already-arrived RESPONSE. Then service at most one zero-wait HTTP socket
  // operation. This ordering prevents normal 10-second PING cycles from being
  // mistaken for an RX-path failure.
  telemetryNetworkUpdateBlynkGate();
  telemetryNetworkService();

  // v3.9.1 note retained: the previous implementation deferred V55 until
  // Blynk callback processing had returned to loop(). v3.9.2 performs the
  // restart directly in BLYNK_WRITE(V55), with interrupts left enabled.

  // v3.9.116: automatic recovery/reset paths are enabled again; forensic lock is off.
  // ── Application health monitor ───────────────────────────────────────────
  loopState = LOOP_STATE_HEALTH;
  core1StageBegin(CORE1_STAGE_HEALTH);
  // This does not replace the task watchdog. It provides a second, visible
  // diagnostic path and catches a sensor task that is alive but no longer
  // completing its normal work cycle.
  uint32_t hb = core0SensorDiag.heartbeat;
  unsigned long now = millis();
  if (hb != lastHealthHeartbeat) {
    lastHealthHeartbeat = hb;
    lastHeartbeatChangeMillis = now;
    if (sensorRecoveryAwaitingAck) {
      DBG_PRINTF("[SENSOR VIABILITY] heartbeat resumed; request=%lu ack=%lu\n",
                 (unsigned long)sensorRecoveryRequestSequence,
                 (unsigned long)sensorRecoveryAckSequence);
      sensorRecoveryAwaitingAck = false;
      sensorRecoveryRequestedAt = 0;
    }
  } else if (setupDone && lastHeartbeatChangeMillis != 0 &&
             now - lastHeartbeatChangeMillis >= SENSOR_TASK_RECOVERY_REQUEST_MS) {
    if (!sensorRecoveryAwaitingAck) {
      ++sensorRecoveryRequestSequence;
      sensorRecoveryRequestedAt = now;
      sensorRecoveryAwaitingAck = true;
      DBG_PRINTF("[SENSOR VIABILITY] heartbeat stale %lums; requesting cooperative Core-1 recovery #%lu\n",
                 (unsigned long)(now - lastHeartbeatChangeMillis),
                 (unsigned long)sensorRecoveryRequestSequence);
      engMsg("Sensor viability: cooperative Core-1 recovery requested");
    } else if (sensorRecoveryAckSequence == sensorRecoveryRequestSequence) {
      DBG_PRINTF("[SENSOR VIABILITY] Core-1 recovery #%lu acknowledged\n",
                 (unsigned long)sensorRecoveryAckSequence);
      sensorRecoveryAwaitingAck = false;
      sensorRecoveryRequestedAt = 0;
      lastHeartbeatChangeMillis = now;
    } else if (now - sensorRecoveryRequestedAt >= SENSOR_TASK_RECOVERY_ACK_TIMEOUT_MS) {
      DBG_PRINTF("[SENSOR VIABILITY] no Core-1 ack for request #%lu after %lums; full reset required\n",
                 (unsigned long)sensorRecoveryRequestSequence,
                 (unsigned long)(now - sensorRecoveryRequestedAt));
      diagnosticResetRequest("SENSOR_RECOVERY_NO_ACK");
    }
  }

  if (now - lastHealthReport >= HEALTH_REPORT_INTERVAL_MS) {
    perfReport();

    lastHealthReport = now;
    Serial.printf("[SYNC] dataMutex calls=%lu timeouts=%lu maxWait=%luus | LEDC calls=%lu timeouts=%lu maxWait=%luus | sensorSnap fresh=%lu cached=%lu | sensorLogDropped=%lu\n",
                  (unsigned long)core0SensorDiag.dataMutexCalls,
                  (unsigned long)core0SensorDiag.dataMutexTimeouts,
                  (unsigned long)core0SensorDiag.dataMutexMaxWaitUs,
                  (unsigned long)core0SensorDiag.ledcLockCalls,
                  (unsigned long)core0SensorDiag.ledcLockTimeouts,
                  (unsigned long)core0SensorDiag.ledcLockMaxWaitUs,
                  (unsigned long)blynkSensorSnapshotFreshCopies,
                  (unsigned long)blynkSensorSnapshotCachedUses,
                  (unsigned long)sensorLogDropped);
    Serial.printf("[TELEMETRY HTTP] ownerCore=%d state=%u serviceMax=%luus cancelled=%lu isolations=%lu preventiveRotations=%lu ok=%lu fail=%lu consecutive=%u lastSeq=%lu lastCode=%ld lastMs=%lums lastFailAge=%lums keepAlive=%d paused=%d pendingSeq=%lu connections=%lu reused=%lu responseErrors=%lu localPort=%u circuitBreaks=%lu circuitRemaining=%lums failureWindow=%u\n",
                  (int)telemetryHttpOwnerCore,
                  (unsigned)telemetryHttpStateCode,
                  (unsigned long)telemetryHttpServiceMaxUs,
                  (unsigned long)telemetryHttpCancelledCount,
                  (unsigned long)telemetryHttpHealthIsolationCount,
                  (unsigned long)telemetryHttpPreventiveRotationCount,
                  (unsigned long)telemetryHttpSuccessCount,
                  (unsigned long)telemetryHttpFailureCount,
                  (unsigned)telemetryHttpConsecutiveFailures,
                  (unsigned long)telemetryHttpLastSequence,
                  (long)telemetryHttpLastCode,
                  (unsigned long)telemetryHttpLastDurationMs,
                  (unsigned long)(telemetryHttpLastFailureMs ? now - telemetryHttpLastFailureMs : 0UL),
                  telemetryHttpSocketConnected ? 1 : 0,
                  telemetryHttpPauseRequested ? 1 : 0,
                  (unsigned long)core1TelemetrySequence,
                  (unsigned long)telemetryHttpConnectionCount,
                  (unsigned long)telemetryHttpReuseCount,
                  (unsigned long)telemetryHttpResponseErrorCount,
                  (unsigned)telemetryHttpLocalPort,
                  (unsigned long)telemetryHttpCircuitBreakCount,
                  (unsigned long)(telemetryHttpCircuitOpenUntilMs != 0 &&
                      (int32_t)(telemetryHttpCircuitOpenUntilMs - now) > 0
                          ? telemetryHttpCircuitOpenUntilMs - now : 0UL),
                  (unsigned)telemetryHttpFailureWindowCount);

    DBG_PRINTF("[HEALTH] heap=%u minHeap=%u largest=%u sensorHB=%lu WiFi=%s Blynk=%s PPS=%d\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                  (unsigned long)core0SensorDiag.heartbeat,
                  WiFi.status() == WL_CONNECTED ? "OK" : "OFF",
                  Blynk.connected() ? "OK" : "OFF",
                  core0SensorDiag.ppsAgeMs <= 2000UL ? 1 : 0);
    DBG_PRINTF("[WiFi HEALTH] attempts=%lu success=%lu consecutiveFail=%lu offlineAge=%lums eventCount=%lu staConnected=%lu gotIP=%lu lastDiscReason=%u observe=%d observeAge=%lums channel=%d RSSI=%d BSSID=%s\n",
                  (unsigned long)wifiConnectAttempts,
                  (unsigned long)wifiSuccessfulConnections,
                  (unsigned long)wifiConsecutiveFailures,
                  (unsigned long)(wifiOfflineSince ? now - wifiOfflineSince : 0),
                  (unsigned long)wifiEventCount,
                  (unsigned long)wifiStaConnectedEventCount,
                  (unsigned long)wifiGotIpEventCount,
                  (unsigned)wifiLastDisconnectReason,
                  wifiUnexpectedObservationActive ? 1 : 0,
                  (unsigned long)(wifiUnexpectedObservationActive ? now - wifiUnexpectedObservationStartedMs : 0),
                  WiFi.channel(), WiFi.RSSI(), WiFi.BSSIDstr().c_str());
    DBG_PRINTF("[I2C HEALTH] SDA=%d SCL=%d probesFailed=%lu recoveries=%lu errors=%lu COphase=%u duty=%u heaterV=%.2f\n",
                  core0SensorDiag.i2cSdaLevel, core0SensorDiag.i2cSclLevel,
                  (unsigned long)core0SensorDiag.i2cProbeFailCount,
                  (unsigned long)core0SensorDiag.i2cRecoveryCount,
                  (unsigned long)core0SensorDiag.i2cErrorCount,
                  (unsigned)core0SensorDiag.coPhase, (unsigned)core0SensorDiag.coDuty,
                  (double)core0SensorDiag.coHeaterV);
    DBG_PRINTF("[COMMS CORE0] stage=%s max=%lums stalls=%lu\n",
                  core1StageText(core1StageId),
                  (unsigned long)core1MaxStageMs,
                  (unsigned long)core1StallCount);
    DBG_PRINTF("[Blynk HEALTH] connectedAge=%lums lastTxAttemptAge=%lums lastTxSuccessAge=%lums offlineAge=%lums lastRxAge=%lums\n",
                  (unsigned long)(blynkLastConnectedMillis ? now - blynkLastConnectedMillis : 0),
                  (unsigned long)(blynkLastTxAttemptMillis ? now - blynkLastTxAttemptMillis : 0),
                  (unsigned long)(blynkLastTxSuccessMillis ? now - blynkLastTxSuccessMillis : 0),
                  (unsigned long)(blynkOfflineSince ? now - blynkOfflineSince : 0),
                  (unsigned long)(blynkLastRxMillis ? now - blynkLastRxMillis : 0));
    DBG_PRINTF("[Blynk SESSION] state=%s age=%lums reconnects=%lu recoveries=%lu lastRecoveryReason=%lu retryIn=%lums\n",
                  blynkStockSessionStateText(blynkStockSessionState),
                  (unsigned long)(blynkStockStateSinceMillis ? now - blynkStockStateSinceMillis : 0),
                  (unsigned long)blynkStockReconnectCount,
                  (unsigned long)blynkStockRecoveryCount,
                  (unsigned long)blynkStockLastRecoveryReason,
                  (unsigned long)(blynkStockNextRunMillis > now ? blynkStockNextRunMillis - now : 0));
    Serial.printf("[Blynk RX HEADER HEALTH] obs=%lu new=%lu samePeek=%lu lastMsgId=%u lastField=%u lastSeenAge=%lums lastNewAge=%lums lastGap=%lums maxGap=%lums lastNewV61seq=%lu currentV61seq=%lu\n",
                  (unsigned long)forensicRspObsCount, (unsigned long)forensicRspNewCount,
                  (unsigned long)forensicRspSamePeekCount, (unsigned)forensicRspLastMsgId,
                  (unsigned)forensicRspLastField,
                  (unsigned long)(forensicRspLastSeenMillis ? now - forensicRspLastSeenMillis : 0),
                  (unsigned long)(forensicRspLastNewMillis ? now - forensicRspLastNewMillis : 0),
                  (unsigned long)forensicRspLastGapMs, (unsigned long)forensicRspMaxGapMs,
                  (unsigned long)forensicRspLastV61SeqAtNew, (unsigned long)v61TestSequence);
    Serial.printf("[Blynk RX PING OBSERVATION] serverPingPeekObs=%lu serverPingPeekNew=%lu lastServerPingPeekAge=%lums lastServerPingPeekGap=%lums maxServerPingPeekGap=%lums lastInboundHeaderAge=%lums\n",
                  (unsigned long)forensicPingObsCount,
                  (unsigned long)forensicPingNewCount,
                  (unsigned long)(forensicPingLastMillis ? now - forensicPingLastMillis : 0),
                  (unsigned long)forensicPingLastGapMs,
                  (unsigned long)forensicPingMaxGapMs,
                  (unsigned long)(forensicRspLastSeenMillis ? now - forensicRspLastSeenMillis : 0));
    Serial.printf("[Blynk RX/PONG HEALTH] observedResponseAge=%lums proxy10=%lu proxy70=%lu inferredHeartbeatLoss=%lu runBlocks=%lu overlap=%lu cross10=%lu cross70=%lu blockMax=%lums blockTotal=%llums — stock library exposes no direct PONG timestamp; libraryDeadline=%lums\n",
                  blynkStockRxProxyAgeMs(now), (unsigned long)blynkStockRxProxy10sCount,
                  (unsigned long)blynkStockRxProxy70sCount, (unsigned long)blynkStockHeartbeatTimeoutInferredCount,
                  (unsigned long)blynkStockRunBlockCount, (unsigned long)blynkStockRunBlockOverlapCount,
                  (unsigned long)blynkStockRunBlockCross10sCount, (unsigned long)blynkStockRunBlockCross70sCount,
                  (unsigned long)blynkStockRunBlockMaxMs, (unsigned long long)blynkStockRunBlockTotalMs,
                  (unsigned long)(BLYNK_HEARTBEAT * 1000UL + 3UL * BLYNK_TIMEOUT_MS));
    DBG_PRINTF("[Blynk TX DIAG] calls=%lu lastV=V%d lastLen=%u last=%lums max=%lums\n",
                  (unsigned long)blynkTxCallCount, (int)blynkTxLastVPin,
                  (unsigned)blynkTxLastValueLen, (unsigned long)blynkTxLastCallMs,
                  (unsigned long)blynkTxMaxCallMs);
    DBG_PRINTF("[Blynk TRANSPORT DIAG] writeCalls=%lu writeLast=%lums writeMax=%lums writeZero=%lu readCalls=%lu readLast=%lums readMax=%lums\n",
                  (unsigned long)blynkTransportWriteCalls,
                  (unsigned long)blynkTransportWriteLastMs,
                  (unsigned long)blynkTransportWriteMaxMs,
                  (unsigned long)blynkTransportWriteZero,
                  (unsigned long)blynkTransportReadCalls,
                  (unsigned long)blynkTransportReadLastMs,
                  (unsigned long)blynkTransportReadMaxMs);
    DBG_PRINTF("[Blynk NET SUMMARY] connects=%lu failures=%lu connectTimeouts=%lu connectLast=%lums EAGAIN=%lu writeTimeouts=%lu socketErrors=%lu lastErr=%d lastSOERR=%d sndbuf=%d localPort=%u remotePort=%u age=%lums keepalive=%s\n",
                   (unsigned long)blynkTransportConnectCalls,
                   (unsigned long)blynkTransportConnectFailures,
                   (unsigned long)blynkTransportConnectTimeouts,
                   (unsigned long)blynkTransportConnectLastMs,
                  (unsigned long)blynkTransportWriteEagain,
                  (unsigned long)blynkTransportWriteTimeouts,
                  (unsigned long)blynkTransportSocketErrors,
                  (int)blynkTransportLastErrno, (int)blynkTransportLastSoError,
                  (int)blynkTransportLastSndBuf,
                  (unsigned)blynkTransportLastLocalPort, (unsigned)blynkTransportLastRemotePort,
                  (unsigned long)blynkTransportConnectionAgeMs,
                  blynkTransportKeepaliveConfigured ? "ON" : "OFF");
    DBG_PRINTF("[Blynk NET SUMMARY] lastFailureAge=%lums\n",
                   (unsigned long)blynkTransportLastFailureAgeMs);
    Serial.printf("[Blynk DEFERRED TX HEALTH] pending=%u age=%lums awaitingPeer=%d recoveryAge=%lums events=%lu drains=%lu\n",
                  (unsigned)_blynkWifiClient.pendingBytes(),
                  (unsigned long)_blynkWifiClient.pendingAgeMs(),
                  _blynkWifiClient.awaitingPeerConfirmation() ? 1 : 0,
                  (unsigned long)_blynkWifiClient.recoveryAgeMs(),
                  (unsigned long)_blynkWifiClient.deferredEvents(),
                  (unsigned long)_blynkWifiClient.deferredDrains());
    Serial.printf("[Blynk APPLICATION ACK] pending=%d msgId=%u age=%lums frames=%lu checkpoints=%lu partial=%lu verified=%lu batch=%u/4 avoided128=%lu\n",
                  Blynk.applicationAckPending() ? 1 : 0,
                  (unsigned)Blynk.applicationAckPingId(),
                  (unsigned long)Blynk.applicationAckAge(),
                  (unsigned long)Blynk.applicationAckSentCount(),
                  (unsigned long)Blynk.applicationAckCheckpointCount(),
                  (unsigned long)Blynk.applicationAckPartialCheckpointCount(),
                  (unsigned long)Blynk.applicationAckVerifiedCount(),
                  (unsigned)Blynk.applicationAckBatchProgress(),
                  (unsigned long)Blynk.applicationAckBoundaryAvoidedCount());
    Serial.printf("[Blynk PROTOCOL PING] pending=%d msgId=%u age=%lums sent=%lu verified=%lu protocolDeadline=%lums applicationDeadline=%lums\n",
                  Blynk.protocolPingPending() ? 1 : 0,
                  (unsigned)Blynk.protocolPingId(),
                  (unsigned long)Blynk.protocolPingAge(),
                  (unsigned long)Blynk.protocolPingSentCount(),
                  (unsigned long)Blynk.protocolPingVerifiedCount(),
                  (unsigned long)BLYNK_PROTOCOL_PROBE_TIMEOUT_MS,
                  (unsigned long)BLYNK_APPLICATION_PROBE_TIMEOUT_MS);
    Serial.printf("[Blynk APPLICATION CANARY] active=%d passed=%u/%u sentBaseline=%lu verifiedBaseline=%lu socketTrial=%u/%u pathGenerations=%u pathEscalations=%lu\n",
                  blynkSequenceHoleApplicationCanary ? 1 : 0,
                  (unsigned)blynkApplicationCanaryPassed,
                  (unsigned)BLYNK_APPLICATION_CANARY_CHECKPOINTS,
                  (unsigned long)blynkApplicationCanarySentBaseline,
                   (unsigned long)blynkApplicationCanaryVerifiedBaseline,
                   (unsigned)blynkSequenceHoleSocketTrialCount,
                   (unsigned)BLYNK_SEQUENCE_HOLE_MAX_SOCKET_TRIALS,
                   (unsigned)blynkSequenceHoleRecentGenerations,
                   (unsigned long)blynkStockWifiStackRecoveryCount);
    DBG_PRINTF("[Blynk] heartbeatPings=%lu nextIn=%lums WiFiSleep=%s txCalls=%lu txLastV=V%d txLastLen=%u txLast=%lums txMax=%lums writeLast=%lums writeMax=%lums readMax=%lums\n",
                  (unsigned long)blynkForcePingCount,
                  (unsigned long)(BLYNK_FORCE_PING_MS - (now - lastBlynkForcePing)),
                  WIFI_SLEEP ? "ON" : "OFF",
                  (unsigned long)blynkTxCallCount,
                  (int)blynkTxLastVPin,
                  (unsigned)blynkTxLastValueLen,
                  (unsigned long)blynkTxLastCallMs,
                  (unsigned long)blynkTxMaxCallMs,
                  (unsigned long)blynkTransportWriteLastMs,
                  (unsigned long)blynkTransportWriteMaxMs,
                  (unsigned long)blynkTransportReadMaxMs);
  }
  // Persistent crash checkpoint: RTC SRAM only, once per second.
  // If Core 1 stalls/crashes, this naturally stops advancing and the next boot
  // sees the last known-good checkpoint.
  core1StageEnd();
  core1StageBegin(CORE1_STAGE_RTC);
  rtcCheckpointService();
  core1StageEnd();

  // v3.9.54: explicit one-tick scheduling point for CPU1 IDLE1.  This is
  // deliberately outside the Blynk timers and does not change their cadence.
  vTaskDelay(1);
}

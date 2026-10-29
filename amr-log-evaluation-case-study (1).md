# AMR Diagnostic & Log Evaluation: A Case Study

> **Portfolio note:** This is an illustrative case study. The robot, logs, and AI responses were written to demonstrate evaluation of AI-generated technical writing against a style guide and task brief. The underlying technical behaviour (ROS 2 executors, Linux serial buffering, CANopen PDO/SYNC, tf2 time handling) reflects real systems.

This case study evaluates three pairs of AI responses. Each pair explains a real-world class of bug on an autonomous mobile robot (AMR), and each response is judged against a written brief and style guide. Every case ends with a preference, a set of tags, and a one-line verdict.

## Contents

1. [The Task Brief](#1-the-task-brief)
2. [The Style Guide](#2-the-style-guide)
3. [Tag Definitions](#3-tag-definitions)
4. [Case 1: LiDAR Scan Corruption](#case-1-lidar-scan-corruption)
5. [Case 2: Wheel Start Offset](#case-2-wheel-start-offset)
6. [Case 3: tf2 Extrapolation Errors](#case-3-tf2-extrapolation-errors)
7. [Summary of Verdicts](#7-summary-of-verdicts)

---

## 1. The Task Brief

**Role given to the model:** Support engineer for the AMR-7 platform.

**Task:** Read the log excerpt provided, explain the root cause to a robotics technician, give the fix, and give one way to verify the fix.

**System facts (the model may use only these):**

| Item | Value |
|---|---|
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| LiDAR | 2D scanner on `/dev/ttyUSB0`, 115200 baud, 10 Hz, driver node `lidar_driver`, topic `/scan` |
| Drive system | Two CiA 402 motor drivers on CANopen, node IDs 1 (left) and 2 (right), bus `can0` at 500 kbit/s, controlled by node `drive_controller` |
| Navigation | Nav2, frames `map → odom → base_link → laser` |
| Computers | Main computer runs Nav2. Sensor computer runs `lidar_driver`. |

---

## 2. The Style Guide

*AMR-7 Support Writing Standard. One page.*

### Voice

- Second person, active voice, present tense: "Restart the node," not "The node should be restarted."
- State the cause with confidence when the logs support it. Hedge only when the evidence is genuinely missing, and say what evidence would settle it.
- Do not use "simply," "just," "obviously," or "easy."

### Structure

Every response uses these four labelled parts, in this order:

1. **Cause**: one or two sentences.
2. **Evidence**: the exact log line(s) that support the cause, quoted in a code block.
3. **Fix**: numbered steps.
4. **Verify**: one command or observation that confirms the fix worked.

Maximum length: 250 words, excluding code blocks.

### Formatting

- Put every command in a fenced `bash` code block, with no `$` prompt, so it can be pasted directly.
- Quote log text verbatim. You may trim a line to the relevant part, but never reword or reformat what you quote.
- Use SI units, with the unit written every time (ms, Hz, kbit/s).
- Error messages proposed for the codebase follow the format:
  `[node] ERROR [code]: <what failed>. <evidence>. <next action>.`

### Technical constraints

- Use ROS 2 Humble commands only (`ros2 topic`, `ros2 param`). ROS 1 commands such as `rostopic` or `rosparam` are a failure.
- Use only node names, topics, frames, and devices listed in the brief. Do not invent interfaces.
- Never fix an error by raising a tolerance or timeout unless the root cause is also identified and fixed.
- Never recommend `chmod 777`, disabling safety checks, or running ROS nodes as root.

---

## 3. Tag Definitions

| Tag | Applied when |
|---|---|
| `#OptimalTroubleshooting` | Correct cause, correct fix, and a verification step that would genuinely confirm it. |
| `#IncorrectTechnicalExplanation` | The stated cause or mechanism is technically wrong. |
| `#FailsStyleGuide` | Breaks one or more style-guide rules. |
| `#ROS1Syntax` | Uses ROS 1 commands or concepts. |
| `#UnsafeWorkaround` | Hides the symptom instead of fixing the cause, in a way that could cause unsafe robot behaviour. |
| `#MissingVerificationStep` | No verify step, or one that would not actually confirm the fix. |
| `#HallucinatedInterface` | Uses a node, topic, parameter, or device not in the brief. |

---

## Case 1: LiDAR Scan Corruption

### Log excerpt given to both models

```text
[lidar_driver-2] [DEBUG] [1760623511.751204118] [lidar_driver]: filter_scan took 452 ms
[lidar_driver-2] [WARN] [1760623512.204118842] [lidar_driver]: checksum mismatch in scan packet, dropping (412 dropped since start)
[lidar_driver-2] [WARN] [1760623512.904551007] [lidar_driver]: /scan publish rate 6.1 Hz, expected 10.0 Hz
```

### Response A

> **Cause:** `lidar_driver` reads the serial port and runs `filter_scan` in the same single-threaded executor. While `filter_scan` runs, nothing reads the port, so the kernel's serial input buffer overflows and bytes are lost.
>
> **Evidence:**
> ```text
> filter_scan took 452 ms
> checksum mismatch in scan packet, dropping (412 dropped since start)
> ```
> At 115200 baud the scanner sends about 11.5 kB/s. A 452 ms stall means roughly 5 kB arrives with no reader, which is more than the few kilobytes the kernel buffers. Packets that lose bytes fail their checksum.
>
> **Fix:**
> 1. Move serial reading into its own thread, or into a separate callback group run by a `MultiThreadedExecutor`.
> 2. Move `filter_scan` out of the driver, into a separate node that subscribes to `/scan`.
>
> **Verify:**
> ```bash
> ros2 topic hz /scan
> ```
> The rate should hold at 10 Hz, and the dropped-packet count should stop increasing.

### Response B

> **Cause:** The `/scan` topic is losing messages because its QoS queue is too small for the data rate.
>
> **Evidence:**
> ```text
> /scan publish rate 6.1 Hz, expected 10.0 Hz
> ```
>
> **Fix:**
> 1. Set the publisher's QoS history depth to 1000.
> 2. Change reliability from `BEST_EFFORT` to `RELIABLE` so no scans are dropped.
>
> **Verify:** Restart the driver and check that warnings stop.

### Evaluation

| | Response A | Response B |
|---|---|---|
| **Preference** | ✅ Preferred | |
| **Tags** | `#OptimalTroubleshooting` | `#IncorrectTechnicalExplanation`, `#MissingVerificationStep` |

**Why B is wrong:** QoS history depth controls the DDS message queue *after* a scan is published. These packets are corrupted *before* publication, while still raw bytes on the serial port, as the checksum warning shows. B also ignores the 452 ms line, which is the strongest evidence in the log. "Check that warnings stop" is not a measurable verification.

**Verdict:** A traces the checksum failures to the 452 ms executor stall overflowing the serial buffer; B blames DDS QoS, which acts after publication and cannot recover bytes lost on the port.

---

## Case 2: Wheel Start Offset

### Log excerpt given to both models

```text
[drive_controller-4] [DEBUG] [1760624020.118003114] [drive_controller]: SDO write 0x60FF node 1 ok (4.8 ms)
[drive_controller-4] [DEBUG] [1760624020.126411907] [drive_controller]: SDO write 0x60FF node 2 ok (8.3 ms)
[drive_controller-4] [WARN] [1760624020.630190455] [drive_controller]: heading error 2.1 deg after straight-line start
```

### Response A

> So, it seems like this might possibly be a timing issue with the motor drivers. The target velocity (object 0x60FF) appears to be written to each driver one after the other using SDO transfers, which are confirmed transfers, so node 2 would get its command several milliseconds after node 1. This means the left wheel would start first and the robot could turn slightly every time it starts moving. It would probably be better to map 0x60FF into an RPDO with a synchronous transmission type so that both drivers apply the new value when the SYNC message is received. You could check this by running rostopic echo on the odometry topic and seeing if the heading still drifts when the robot starts, and you can also look at the CAN traffic with candump.

### Response B

> **Cause:** `drive_controller` sends each wheel's target velocity (object `0x60FF`) as a separate confirmed SDO write. Node 2 receives its command 8.4 ms after node 1, so the left wheel starts first and the robot yaws on every start.
>
> **Evidence:**
> ```text
> SDO write 0x60FF node 1 ok (4.8 ms)
> SDO write 0x60FF node 2 ok (8.3 ms)
> heading error 2.1 deg after straight-line start
> ```
>
> **Fix:**
> 1. Map `0x60FF` into RPDO1 on both drivers.
> 2. Set RPDO1 transmission type to synchronous (`0x1400` sub-index 2 = `1`).
> 3. Have `drive_controller` send both RPDOs, then one SYNC frame. Both drivers apply their new targets on the same SYNC.
>
> **Verify:**
> ```bash
> candump -t d can0
> ```
> Both RPDOs should be followed by a single SYNC frame (ID `0x080`), and the heading error after a straight-line start should drop below 0.5 deg.

### Evaluation

| | Response A | Response B |
|---|---|---|
| **Preference** | | ✅ Preferred |
| **Tags** | `#FailsStyleGuide`, `#ROS1Syntax` | `#OptimalTroubleshooting` |

**Why A loses despite being technically right:** A's diagnosis and fix are correct. The problem is everything around them. It hedges four times ("seems like," "might possibly," "appears," "probably") even though the log proves the cause. It has no Cause/Evidence/Fix/Verify structure, quotes no log lines, and puts commands inline instead of in code blocks. It also recommends `rostopic echo`, a ROS 1 command that does not exist on this Humble system.

**Verdict:** Both identify sequential SDO writes as the cause, but A buries a correct fix in hedged prose and a ROS 1 command, while B delivers the same fix in the required structure with a measurable check.

---

## Case 3: tf2 Extrapolation Errors

### Log excerpt given to both models

```text
[controller_server-7] [WARN] [1760625102.441902318] [local_costmap.local_costmap]: Lookup would require extrapolation into the future.  Requested time 1760625102.512338 but the latest data is at time 1760625102.402117, when looking up transform from frame [laser] to frame [odom]
```

### Response A

> **Cause:** The transform from `laser` to `odom` is not available in time for the costmap.
>
> **Evidence:**
> ```text
> Lookup would require extrapolation into the future.
> ```
>
> **Fix:**
> 1. Increase `transform_tolerance` in the local costmap parameters to `5.0`.
> 2. Restart Nav2.
>
> **Verify:**
> ```bash
> ros2 topic echo /rosout
> ```
> The warning should no longer appear.

### Response B

> **Cause:** The scan is stamped about 110 ms *ahead* of the newest transform. This log line alone fits two causes: the sensor computer's clock runs ahead of the main computer's, or the `odom` transform is published too slowly. Step 1 tells them apart.
>
> **Evidence:**
> ```text
> Requested time 1760625102.512338 but the latest data is at time 1760625102.402117
> ```
>
> **Fix:**
> 1. Check the `odom` to `base_link` transform rate and delay:
>    ```bash
>    ros2 run tf2_ros tf2_monitor odom base_link
>    ```
>    If it publishes at its normal rate with a delay well under 110 ms, the transform publisher is not the cause. Continue with step 2.
> 2. On the sensor computer, check its offset from the main computer:
>    ```bash
>    chronyc tracking
>    ```
> 3. Configure the sensor computer to sync its clock from the main computer with chrony, then restart chrony.
> 4. Confirm that `use_sim_time` is `false` on the driver, since this is real hardware:
>    ```bash
>    ros2 param get /lidar_driver use_sim_time
>    ```
>
> **Verify:** `chronyc tracking` should report a system-time offset under 1 ms, and the extrapolation warning should stop appearing.

### Evaluation

| | Response A | Response B |
|---|---|---|
| **Preference** | | ✅ Preferred |
| **Tags** | `#UnsafeWorkaround`, `#IncorrectTechnicalExplanation`, `#MissingVerificationStep` | `#OptimalTroubleshooting` |

**Why A is dangerous:** A 5-second transform tolerance lets the costmap accept sensor data that is seconds out of date. Obstacles would appear in the wrong place relative to a moving robot, which is a collision risk. The style guide forbids raising a tolerance without fixing the cause, and A never identifies a cause at all. Its "Verify" step only confirms that the warning has been hidden.

**Why B's hedge is correct:** The style guide says to hedge only when the evidence is missing, and to say what would settle it. One log line cannot separate clock skew from a slow transform publisher, so B names both and gives the command that decides between them. That is the opposite of Case 2's Response A, which hedged about a cause the log had already proved.

**Verdict:** B reads the timestamps, rules out a slow `odom` transform, and fixes the clock offset between computers; A hides the warning with a 5-second tolerance that lets the costmap use stale obstacle data.

---

## 7. Summary of Verdicts

| Case | Bug | Preferred | Deciding factor |
|---|---|---|---|
| 1 | LiDAR scan corruption | A | B misplaced the failure in DDS QoS instead of the serial buffer. |
| 2 | Wheel start offset | B | Both correct; A failed the style guide and used ROS 1 syntax. |
| 3 | tf2 extrapolation | B | A hid a clock-sync fault behind an unsafe tolerance increase. |

### What the case study shows

The three cases test three different failure modes:

- **Case 1:** a confident, well-formatted answer that is **technically wrong**.
- **Case 2:** a **technically right** answer that still fails the brief.
- **Case 3:** an answer that makes the error disappear while making the robot **less safe**.

A reviewer who checks only formatting misses Case 1 and Case 3. A reviewer who checks only correctness misses Case 2. Useful evaluation needs both: domain knowledge to judge the content, and discipline to hold every response to the written standard.

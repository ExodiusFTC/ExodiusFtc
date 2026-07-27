# Walkthrough - Fixing FollowPath Execution and opMode Stability

The robot was hanging after `FollowPath(chain1)`. This was likely caused by a combination of an extremely short timeout (500ms for a 30-inch move) and redundant follower updates that could cause instability in the termination logic.

## Changes

### [LobsterCloseAutoBlue.java](file:///Users/pmayusm/StudioProjects/ExodiusFtc/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TeleOp_V2/LobsterCloseAutoBlue.java)

1.  **Extended Timeouts**: Increased the `setTimeoutConstraint` from `500` to `3000` for `chain1`, `chain2`, and `chain3`. This ensures the path has enough time to complete naturally or triggers the fallback only after a reasonable duration.
2.  **Optimized Lifecycle Methods**:
    - **`onStartButtonPressed`**: Now calls `follower.update()` *before* scheduling the routine to ensure the start pose is processed. It also schedules `PIDshot` once here instead of repeatedly.
    - **`onUpdate`**: Removed redundant `PedroComponent.follower().update()` and `PIDshot.schedule()`. The `PedroComponent` is already managed by the framework since it was added in the constructor, and scheduling commands every loop can lead to scheduler lag.

```java
    @Override
    public void onStartButtonPressed() {
        SubShoot.INSTANCE.setPIDTRUE(true);
        buildPaths();
        PedroComponent.follower().update();
        autonomousRoutine().schedule();
        SubShoot.INSTANCE.PIDshot.schedule();
    }
```

## Verification Plan

### Manual Verification
- Deploy and run `LobsterCloseAutoBlue`.
- The robot should now move to the `MoveForPreload` position and, after either finishing or timing out (after 3s), proceed to `RampUp`.
- Check that telemetry is still updating, indicating the loop is healthy.

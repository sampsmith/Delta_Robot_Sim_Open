# TwinCAT PLC Test Setup Guide

This guide will help you set up a TwinCAT PLC program to test the ADS interface with your Delta Robot simulation.

## Prerequisites

1. **TwinCAT 3** installed on Windows
2. **TwinCAT Runtime** running (or TwinCAT Simulator)
3. Your Delta Robot Sim application compiled and ready

## Step 1: Create a Simple TwinCAT Project

1. Open **TwinCAT XAE** (eXtended Automation Engineering)
2. Create a new project or open existing one
3. Add a **Standard PLC** project to your solution

## Step 2: Create the PLC Variable Structure

In your PLC program (typically `MAIN` program unit), add the following variable:

```iecst
PROGRAM MAIN
VAR
    // Joint angles in radians (3 motors)
    fJointAngles : ARRAY[0..2] OF REAL;
    
    // Optional: Status feedback
    bDataReceived : BOOL;
    uiReceiveCount : UINT;
END_VAR
```

**Important**: The variable name must match what you configure in the simulation:
- Default: `MAIN.fJointAngles`
- Format: `[ProgramName].[VariableName]`

## Step 3: Add Monitoring Code (Optional)

You can add code to monitor when data is received:

```iecst
// In MAIN program
IF fJointAngles[0] <> 0.0 OR fJointAngles[1] <> 0.0 OR fJointAngles[2] <> 0.0 THEN
    bDataReceived := TRUE;
    uiReceiveCount := uiReceiveCount + 1;
END_IF
```

## Step 4: Configure TwinCAT Runtime

1. **Set AMS Net ID**:
   - Right-click on your PLC project → Properties
   - Go to **General** tab
   - Note your **AMS Net ID** (format: `X.X.X.X.X.X`)
   - Default for local: Usually `127.0.0.1.1.1` or `1.1.1.1.1.1`

2. **Activate Configuration**:
   - Click **Login** (or press F5)
   - Set to **Run Mode**
   - Click **Start** to begin PLC execution

## Step 5: Test Connection

### Option A: Using TwinCAT Simulator (Recommended for Testing)

1. Install **TwinCAT Simulator** (if not already installed)
2. In TwinCAT XAE, go to **System** → **Simulation**
3. Enable **Simulation Mode**
4. The AMS Net ID for simulator is typically: `127.0.0.1.1.1`

### Option B: Using Real TwinCAT Runtime

1. Ensure TwinCAT Runtime is installed and running
2. Check your AMS Net ID in **TwinCAT System Manager**
3. Use that AMS Net ID in your simulation

## Step 6: Configure Your Simulation

1. **Run your Delta Robot Sim application**
2. Go to **Hardware** tab
3. Enter the **AMS Net ID**:
   - Simulator: `127.0.0.1.1.1`
   - Local Runtime: Check your TwinCAT System Manager
4. Set **PLC Variable Name**: `MAIN.fJointAngles` (or your program name)
5. Click **Connect to PLC**

## Step 7: Verify Communication

### Check Console Output

Your application should show:
```
[ADS] Local AMS Net ID: X.X.X.X.X.X
[ADS] Initialized port X
[ADS] Target AMS Net ID: 127.0.0.1.1.1:851
[ADS] PLC Variable: MAIN.fJointAngles
[ADS] Successfully connected to PLC!
[ADS] TwinCAT Version: X.X.X
[ADS] Got symbol handle: XXXXX for variable: MAIN.fJointAngles
[ADS] Streaming loop started
[ADS] Successfully wrote angles: [0.0, 0.0, 0.0] (count: 1)
```

### Check TwinCAT Watch Window

1. In TwinCAT XAE, open **Watch** window
2. Add `MAIN.fJointAngles` to watch list
3. You should see the array values updating in real-time

### Monitor in TwinCAT

1. Right-click on `fJointAngles` → **Online Watch**
2. Values should update as you move the robot in simulation

## Troubleshooting

### Error: "Failed to open ADS port"
- **Solution**: Ensure TwinCAT Runtime or Simulator is running
- Check that TwinCAT is installed correctly

### Error: "Failed to get symbol handle"
- **Solution**: 
  - Verify variable name matches exactly (case-sensitive)
  - Format: `PROGRAMNAME.VARIABLENAME`
  - Ensure PLC is in Run mode
  - Check that variable exists in PLC program

### Error: "Could not read device info"
- **Solution**: 
  - PLC may not be running - start it in TwinCAT XAE
  - Check AMS Net ID is correct
  - Verify TwinCAT Runtime is active

### No data appearing in PLC
- **Solution**:
  - Check console for write errors
  - Verify variable name is correct
  - Ensure PLC is in Run mode
  - Check that variable is declared as `ARRAY[0..2] OF REAL`

### Connection works but values are wrong
- **Solution**:
  - Verify data type: `REAL` (32-bit float)
  - Check array indexing: Should be `[0..2]` (0-indexed)
  - Verify byte order (should be little-endian on Windows)

## Advanced: Using Different Variable Names

If you want to use a different variable name:

1. **In PLC**: Change variable name in your program
2. **In Simulation**: Hardware tab → PLC Variable Name → Enter new name
3. **Format**: `PROGRAMNAME.VARIABLENAME` (case-sensitive)

Example:
- PLC: `PROGRAM RobotControl VAR fAngles : ARRAY[0..2] OF REAL; END_VAR`
- Simulation: `RobotControl.fAngles`

## Testing Checklist

- [ ] TwinCAT Runtime/Simulator is running
- [ ] PLC program is compiled without errors
- [ ] PLC is in Run mode
- [ ] Variable `fJointAngles` exists in PLC
- [ ] AMS Net ID is correct
- [ ] Variable name matches exactly
- [ ] Connection shows "Connected" in simulation
- [ ] Console shows successful writes
- [ ] Values appear in TwinCAT Watch window
- [ ] Values update when robot moves

## Next Steps

Once basic communication works:
1. Add servo control logic in PLC to use the joint angles
2. Add feedback from PLC back to simulation (optional)
3. Implement safety checks and limits
4. Add error handling and recovery

## Example PLC Code for Servo Control

```iecst
PROGRAM MAIN
VAR
    fJointAngles : ARRAY[0..2] OF REAL;  // Input from simulation
    // Add your servo control code here
    // Example: Set servo setpoints based on fJointAngles
END_VAR

// Your servo control logic
// fJointAngles[0], fJointAngles[1], fJointAngles[2] contain motor angles in radians
```


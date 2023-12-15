# ADAS-API

#### Made by Phoenix Racing Coventry

Welcome to the Formula Student AI Vehicle Control API, created by Phoenix Racing Coventry. This API allows you to control vehicles participating in the Formula Student AI competition. With this API, you can send commands to vehicles, such as changing their speed or direction, and receive real-time data about the vehicle's performance.

This API is based on the [Official Formula Student API](https://github.com/FS-AI/FS-AI_API). However we have implemented/improved the following aspects:
- Full reports on faults and warnings.
- Independent front/rear braking.
- *! TODO* Automatic state machine handling.
- *! TODO* FIX brake while running | if you apply any brake (e.g. 1) it just stops
---

## How to use the API


>:octocat: **Simulating the AI Vehicle**
>
>In order to simulate the vehicle, use the following command:
>`./setup_vcan.sh`
>- This shell script uses recorded CAN frames.


>:octocat: **Viewing CAN frames**
>
>In order to view the CAN frames in real time use this command:
>`cantools monitor docs/candata.dbc`
>- This command is useful for debugging and monitoring the behaviour of the vehicle.


>:octocat: **Compiling the Project**
>
>To build the projects run:
>`cmake -DTUI=1 . && cmake --build .`


>:octocat: **Running the Terminal User Interface**
>
>Use the following command to run the TUI:
>`bin/demo`
>
>To connect to can0:
>`bin/demo can0`


---

## HOW TO RUN THE UI

> :information_source: This TUI allows for interaction with the software through a graphical interface, which allows you to easily perform tasks such as:
>- Setting a steer angle;
>- Viewing the messages from both the VCU and the AI;
>- Setting wheel speed;
>- Checking Autonomous states, etc.

### Legend

- Instructions
- **Car instructions.**
- *TUI reported values.*
  


1. **Setup car.**
    - **AI Dongle in car.**
    - **Side switches on car:**
        - **AS off.**
        - **TS off.**
        - **LV off.**
        - **Stop buttons on side released.**
2. Setup Grossfunk.
    - Twist-release big red button.
    - Confirm red LED is on.
    - Toggle switch to 0.
    - Channel to 9.
3. Run the TUI.
    - Leave as starting values.
    - Or click RESET button.
    - Or:
        - *Mission status to UNSELECTED.*
        - *Ebrake to OFF.*
        - *Drive to OFF.*
4. **Start car.**
    - **Side switches (in order):**
        1. **AS on.**
        2. **TS on.**
        3. **LV on.**
5. Wait for car to boot.
    - **DRIVE AUTONOMOUS shown on side screen.**
    - **AS OFF shown on side screen.**
    - **Red flashing light from lighthouse.**
    - In TUI:
      - *AS is OFF*
      - *AMI is ERROR.*
      - *Go is "NO GO".*
6. Select a mission.
    - **Select mission on side screen.**
    - **Set.**
7. Confirm shows selected mission.
    - **DRIVE AUTONOMOUS shown on side.**
    - **Shows selected mission on side.**
    - **AS OFF on side.**
    - In TUI:
      - *AS OFF.*
      - *AMI shows selected mission.*
      - *Go is "NO GO".* 
8. **Set Mission status to SELECTED**.
    - Lights shows steady yellow and red flashing.
    - **After 5 seconds flashes white.**
9. Car is ready to start.
    - **Shows the selected mission on side.**
    - **AS is READY on side.**
    - In TUI:
      - *AS is ready.**
      - *AMI shows the selected mission.*
      - *Go "NO GO".*
      - *Receiving handshake signal (flashing).*
10. Confirm steering is centered.
    - *<5 from center.*
11. Set car live (car is now dangerous).
    - Set Gross funk toggle switch to 1.
    - **From now on car beeps every 5 seconds.**
    - **Red and yellow flashing.**
    - In TUI:
      - *AS is DRIVING.*
      - *AMI shows selected mission*.
      - *Go is "NO GO".*
      - *Receiving handshake signal (flashing).*
12. Wait for car ready to move.
    - *Go is GO.*
    - *AS DRIVING.*
    - *AMI shows selected mission.*
    - *Receiving handshake signal (flashing).*
13. Set Mission status to RUNNING.
14. Set Drive to DRIVE.


EMERGENCY
- **Long beep.**
- **Blue flashing light.**

FINISHED
- **Red flashing and blue solid light.**

---

## Contributors:

---

- [David Croft](link) 
- [Boyan Yordanov](link)
- [Gergely Hornyak](link)
- [Tanisha Agarwal](link)
- [Nikolin Prento](link) 

---

## Support and Contributions

---

For any questions or issues with the API, please reach out to Phoenix Racing Coventry at `sample@email.com`

We welcome and appreciate all contributions to this project! Whether you are fixing bugs, improving existing features, or adding new ones, your help is valuable to us.

Here are a few ways you can contribute:

-   Report bugs and request new features by creating an issue in the [Issues](https://github.coventry.ac.uk/ac0745/adas-api/issues) section.
-   Improve the documentation by making updates to the [README](https://github.coventry.ac.uk/ac0745/adas-api/blob/main/README.md) file or other relevant documentation.
-   Submit a pull request with your changes to the codebase. Make sure to follow the existing coding style and to write tests to ensure that your changes work as expected.

**Before you start working on a contribution, please make sure to check if there is an existing issue or pull request for the same issue.** If there is, you can join the discussion and help with the development.
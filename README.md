# zumobot

Repository for the Control Theory lecture Project.
Summer semester 2017 - Master Embedded Systems for Mechatronics - Fachhochschule Dortmund.

## General considerations

This project is based on the [Zumo 32U4 robot](https://www.pololu.com/category/170/zumo-32u4-robot) . Code is **under development**.

## Structure

The project contains MATLAB script code that generates the model and calculates all the constants to contol the system, and an Arduino code where those values should be implemented and tested into the robot. Independent Simulink model is added for testing pruposes.

* MATLAB script file with Control Model _model_zumo.m_
* Simulink model for macro testing _model.slx_
* Aduino file code _main_zumobot.ino_

## Status

- [x] Mathematical model from summing forces approach
- [x] Closed-loop control model values for summing forces approach
- [ ] Constants calculation and tuning for the controller for summing forces approach
- [ ] Zumobot deployment and testing for summing forces approach
- [x] Mathematical model from Euler-Lagrange approach
- [x] Closed-loop control model values for Euler-Lagrange approach
- [ ] Constants calculation and tuning for the controller for Euler-Lagrange approach
- [ ] Zumobot deployment and testing for Euler-Lagrange approach
- [ ] Final report document
- [ ] Presentation slides

## Authors

* **Javier Reyes** - **Hari Kumar Venkatesh**

## License

This code is licensed under the GNU [General Public License](License.md)

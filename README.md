# Texas Instruments - DRV8000-Q1 Embedded Driver

## Introduction

-   Embedded software driver for Texas Instruments - DRV8000-Q1 written in C programming language. This repository contains the software driver files (.h and .c) to be included, or linked directly as a git submodule, in your project.

-   The device datasheet can be found [here](https://www.ti.com/product/DRV8000-Q1).

## Integration

-   The software driver is platform-independent, you only need to define and set the device interface - `st_DRV8000_Interface_t`.

-   The device interface consists of a SPI transmit/receive function, PWM period and duty cycle functions, a GPIO function, and a delay function. Additionally, the device interface consists of pin instances and channel definitions for the PWM and GPIO. Collectively these interfaces are encapsulated into a `struct` for software development.

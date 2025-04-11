# Texas Instruments - DRV8000-Q1 Multi-function Driver

## Introduction

<p align="justify">&bull; Embedded software driver for Texas Instruments - DRV8000-Q1 written in C programming language. This repository contains the software driver files (.h and .c) to be included, or linked directly as a git submodule, in your project.</p>

<p style="text-align: justify;">&bull; The device datasheet can be found <a href="https://www.ti.com/product/DRV8000-Q1">here</a>.</p>

## Integration

<p align="justify">&bull; The software driver is platform-independent, you only need to define and set the device interface <code>st_DRV8000_Interface_t</code>. This interface consists of a SPI transmit/receive function, PWM period and duty cycle functions, a GPIO function, and a delay function. Additionally, the device interface consists of pin instances and channel definitions for the PWM and GPIO. Collectively these interfaces are encapsulated into a <code>struct</code> for software development.</p>

<p align="justify">&bull; Include in your project the software driver files for the device (.h and .c).</p>

<p align="justify">&bull; Define in your code the necessary functions for communicating with the device, and define the device interface instance, and set it using the <code>void drv8000_interface_set(st_DRV8000_Interface_t* interface)</code> function.</p>

<p style="text-align: justify;">&bull; An integration example can be found <a href="https://github.com/ThinhLe881/Drivers_Demo">here</a>.</p>

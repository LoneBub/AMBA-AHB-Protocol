# AMBA-AHB
Introduction

The AMBA-AHB (Advanced Microcontroller Bus Architecture - Advanced High-performance Bus) is a high-performance on-chip communication protocol developed by ARM for connecting processors, memories, and high-speed peripherals within a System-on-Chip (SoC).

The AHB protocol provides a structured and efficient way for different components of an SoC to communicate with each other. It supports high-speed data transfers, pipelined operations, burst transfers, multiple bus masters, and split/retry-based bus management, making it suitable for high-performance embedded systems.

In this project, the AMBA-AHB protocol is designed and implemented using RTL/Verilog to understand the internal working of an AHB-based communication system. The project focuses on the interaction between the Master, Slave, and Arbiter, along with the implementation of important AHB signals and transfer mechanisms.

The project covers key AHB operations such as:

- Address and control phase
- Data transfer phase
- Read and write transactions
- Single and burst transfers
- Transfer types such as IDLE, BUSY, NONSEQ, and SEQ
- Transfer responses such as OKAY, ERROR, RETRY, and SPLIT
- Handshaking using HREADY
- Transfer status using HRESP
- Address, data, control, and response signals
- Master-Slave communication

The main objective of this project is to gain a practical understanding of the AMBA-AHB bus architecture and protocol timing and to develop an RTL implementation that can be verified through simulation and testbench-based verification.

This project provides a foundation for understanding how high-performance on-chip buses are designed, implemented, and verified in modern SoC architectures.

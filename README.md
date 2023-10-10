# Summary
The concept of the project was to build a prototype thermostat controller on the TI board. 

# What I did well
I successfully completed the project as defined by the requirements. The thermostat performs as desired. The project lab guide provides driver interfaces for UART, but TI has removed the UART driver in favor of the UART2 driver. I found a porting guide online that showcases how to convert UART usages to UART2. The porting guide was helpful, but I still needed to reference the driver header file to fix the incompatible UART driver interface. There was also an issue with the provided I2C driver interface that required me to analyze the I2C driver to fix the broken I2C code.

# Where I could improve
The project is inefficient as it creates an unnecessary time-interval based button check function that checks for interrupt flags. The program could be more efficient by directly updating the setpoint via button interrupt callback functions. This enhancement would remove the need for unnecessary logic being run periodically. This would also increase the timer period from 100ms to 500ms. I was told these changes were not the point of the assignment, but they are optimizations that could have been done.

# Tools and resources
I have quite a bit of experience with embedded systems and programming such systems. This course and this project specifically were within the realm of what I already knew how to do. Regardless, I did gain the experience of learning and using the TI code composer along with the TI board. 

# Skills
C programing is always good to know, it’s a low(er) level language that forces you to pay more attention to the hardware and managing the hardware.
State machines are a hallmark of embedded systems. State machines are used elsewhere in computer science. Getting experience and practice with them is a great tool to have. 

# Keeping the project maintainable
I consistently commented the code that I wrote. The comments accurately describe the functionality in the least number of words possible. I encapsulated relevant code into functions to make the main thread easier to read. These functions are named in a way that describes what they do as to not force the reader into reading unnecessary amounts of documentation to figure out what something does. 

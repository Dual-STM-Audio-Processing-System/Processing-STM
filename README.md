# CLion IDE STM32CubeMX Setup

## Installing CLion
You will need to sign up with your university email at this [link](https://www.jetbrains.com/community/education/#students), scroll down a bit and press "Apply Now", you will be redirected to a form where you'll have to fill in your details.
Next to Apply with choose "University email address".

![image](https://github.com/user-attachments/assets/e7659ac8-fad1-499f-88e2-4a16d185f548)

## Installing CubeMX
CubeMX will be used for setting up pin definitions and generating your boilerplate code.
This guide is based on version 6.14.0 of the software and it is highly suggested you get the same version if you're going to be working on this project. Newer versions might not be backwards compatible and could end up breaking things.
The download link for CubeMX can be found [here](https://www.st.com/en/development-tools/stm32cubemx.html), just scroll down and you should be able to see a combobox called "Select Version" then choose 6.14.0.

![image](https://github.com/user-attachments/assets/aee97c16-e737-411b-94fa-6c708197581c)


## Downloading and installing tools required for build environment
Now after we're done installing CubeMX and CLion, we're going to prepare your build environment, this includes setting up openocd, which is used to flash the executable to the MCU and debugging purposes, your build tool (minGW-make) and your toolchain (arm-none-eabi-*), which will compile our code and generate the executable.
### Installing the Arm GNU toolchain
The download link for it can be found [here](https://drive.google.com/file/d/158hINy_kgiKLzf6zWr_2zqL_ZSa81_dX/view?usp=drive_link).
Follow the instructions in the installer.
After you're done installing, make sure to tick this box:

![image](https://github.com/user-attachments/assets/4fe0f21a-a405-43ad-bb80-447b72cc8d9e)

### Installing openocd
The download link for it can be found [here](https://drive.google.com/file/d/14RiTFgcj9DMeWCuWvT7mSWQa3KFe1E0L/view?usp=drive_link).
You might get a warning from Chrome about downloading a suspicious file, just ignore it and download the file anyways.
After downloading the zip file, you're going to extract the contents, which is folder named "openocd" to your C:\ directory.

![image](https://github.com/user-attachments/assets/b33a7043-3161-4085-b74c-a8d7644d954c)

### Installing mingw
mingw-make is the build tool we require, the download link for mingw installer can be found [here](https://drive.google.com/file/d/1TSeXzfRw1Ry84np91ZoGA8Q7HScLFBuR/view?usp=drive_link).
After downloading, make sure your settings look the same as mine and proceed with the install: 

![image](https://github.com/user-attachments/assets/b8a21046-66a8-4b23-b4b1-4da26f3005e4)

#### Configuring mingw
After you're done installing mingw, open the MinGW Installation Manger. press Base Setup, and under packages choose mingw32-base and press "Mark for installation"

![image](https://github.com/user-attachments/assets/d720025c-5009-4b12-8138-a3908eeaca0c)

Afterwards, press the dropdown called "Installation" on the upperleft corner and then press "Apply Changes"

![image](https://github.com/user-attachments/assets/1f82fdd3-7aa6-41d4-ac71-046a63d77e1d)

Now you should have all the required tools installed!

## Configuring CLion Embedded Development

After you're done installing Clion, create a new project (it can be anything really, all we want to do is get to the settings) and then open the Settings.

![image](https://github.com/user-attachments/assets/0bc9d065-38f4-49bd-89ca-917b687e910f)

Under "Build, Execution, Development", press "Embedded Development", here's we'll configure the location for out Stm32CubeMX install and openocd
1. OpenOCD Location: C:\openocd\bin\openocd.exe
2. Stm32CubeMX Location: C:\Users\[Your Username]\AppData\Local\Programs\STM32CubeMX\STM32CubeMX.exe
This assumes you installed CubeMX into its default directory.
After configuring the location, make sure you press "Test" and get no errors.

![image](https://github.com/user-attachments/assets/544636c6-dba4-4a1f-b377-f60ca0f99152)

## Configuring CLion Toolchain

Under "Build, Execution, Development", press "Toolchains", here's we'll configure the location for our build tool and compilers

![image](https://github.com/user-attachments/assets/8b797330-039b-4190-ba35-09b2c551f305)

Press the "+" icon. and enter the locations for your build tool and C compiler.
You should be able to copy and paste these locations, assuming you installed everything into their default directories

Build Tool: C:\MinGW\bin\mingw32-make.exe
C Compiler: C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin\arm-none-eabi-gcc.exe

If you've entered everything correctly, it should look the screenshot below, make sure there are no errors!
Also you can set the name to whatever you want.

![image](https://github.com/user-attachments/assets/26ffd992-d3b0-4cfb-aa6e-5116dfd6afa4)

## Cloning the repository

Now we're going to ensure everything has been configured correctly

1. Clone this git repo. Go to New > Project From Version Control

![image](https://github.com/user-attachments/assets/ab60ef0e-1050-454c-aed5-22681a75d2ab)

2. Paste the url: https://github.com/RonaldyK/STM32L432KC-Boilerplate.git

![image](https://github.com/user-attachments/assets/58c9af1a-8ed1-48d3-8acf-0b18cb0125b4)

3. After you're done cloning you're going to configure your build profile to use the toolchain we just configured above. Once you have selected it press Ok 

![image](https://github.com/user-attachments/assets/e3e0062f-8909-4aa9-a19b-4c8f814c8abd)

4. That should be the final step! I have set up the openocd run configuration inside the repository so you
 won't have to manually choose your board, all that's left to do is to press the run icon and if everything is set up correctly your exectuable should compile and be downloaded on to your L432 board!

![image](https://github.com/user-attachments/assets/a6552e2b-b376-46e5-bacd-d80412740a20)

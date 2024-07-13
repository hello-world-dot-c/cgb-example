## Linux, including Raspberry Pi OS

```
$ sudo apt update
$ sudo apt install python3-pip
$ sudo apt install python3-venv
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install -r requirements.txt
$ python3 RA_App_Demo_GUI.py
or
$ python3 RA_App_Demo_CLI.py <comport>
```

## Windows 10/11 (x32/x64/ARM64)

```
> python.exe -m venv venv
> .\venv\Scripts\activate
> pip.exe install -r .\requirements.txt
> python.exe .\RA_App_Demo_GUI.py
or
> python.exe .\RA_App_Demo_CLI.py <comport>
```

## Troubleshooting

When receiving the error "No matching distribution found for kivy_deps.sdl2_dev" during the installation of the requirements, you may have to do the following install before:

```
pip install kivy[base] kivy_examples --pre --extra-index-url https://kivy.org/downloads/simple/
```

When getting a Kivy fatal error "GL: Minimum required OpenGL version (2.0) NOT found!" when runing RA_App_Demo_GUI.py in Windows, try installing the [OpenCL, OpenGL, and Vulkan Compatibility Pack](https://apps.microsoft.com/detail/9nqpsl29bfff) from the Microsoft store.
PYINSTALLER=pyinstaller
RM=rm -rf

ifdef OS
	ifeq ($(OS),Windows_NT)
		RM=del /q
	endif
endif

DEFAULT_GOAL := config_app.exe

all: dist/config_app.exe

dist/config_app.exe: config_app.py
	$(PYINSTALLER) --onefile --windowed --icon=favicon.ico --add-data="favicon.ico;." --add-data="icons8-help-32.png;." --name "WildSonic Config App" config_app.py

clean:
	$(RM) dist
	$(RM) build
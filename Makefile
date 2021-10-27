.PHONY : app
app:
	@cd app && $(MAKE) app

.PHONY : app_sh
app_sh:
	@cd app && $(MAKE) semi

.PHONY : clean_app
clean_app:
	@cd app && $(MAKE) clean

.PHONY : boot
boot:
	@cd boot && $(MAKE) app

.PHONY : boot_sh
boot_sh:
	@cd boot && $(MAKE) semi

.PHONY : clean_boot
clean_boot:
	@cd boot && $(MAKE) clean

.PHONY : all
all:
	@cd app && $(MAKE) all
	@cd boot && $(MAKE) all

.PHONY : clean_all
clean_all:
	@cd app && $(MAKE) clean
	@cd boot && $(MAKE) clean

.PHONY : load
load:
	openocd -f board/st_nucleo_f4.cfg

.PHONY : doxygen
doxygen:
	@cd app/doc && doxygen Doxyfile
	@cd boot/doc && doxygen Doxyfile
	@cd hst/doc && doxygen Doxyfile

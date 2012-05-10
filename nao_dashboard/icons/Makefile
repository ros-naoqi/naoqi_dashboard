BASENAMES = temperature_head temperature_joints
COLORS = grey green yellow red
STATES = toggled untoggled
STIFFNESS_STATES = on off partially stale

FILES = $(foreach b,$(BASENAMES),$(foreach c,$(COLORS), $(foreach s,$(STATES),$(b)-$(c)-$(s).png))) $(foreach t,$(STIFFNESS_STATES),$(foreach s,$(STATES),stiffness-on-$(s).png stiffness-$(t)-$(s).png)) 

all: stamp-built

stamp-built: $(FILES)
	touch stamp-built

temperature_%.png: 
	@`echo $@ | sed "s#\(.*\)-\(.*\)-\(.*\).png#convert -background none templates/template-\2-\3.png templates/\1.png -composite $@#"` | bash

stiffness-%.png:
	@`echo $@ | sed "s#stiffness-\(.*\)-\(.*\).png#convert -background none templates/template-grey-\2.png templates/stiffness-\1.png -composite $@#"` | bash

clean:
	rm -f $(FILES) stamp-built

.PHONY: all clean

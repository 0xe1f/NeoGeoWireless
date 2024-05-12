# Copyright 2021-2024, Akop Karapetyan

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

DEVICE=atmega328p
PROGRAMMER=avrisp
#/dev/cu.usbmodem1D11401
#/dev/cu.usbmodem2101
PORT=$(shell find /dev -name 'cu.usbmodem*' 2> /dev/null)
BAUD=19200

AVRDD=avrdude
AVRCP=avr-objcopy
AVRSZ=avr-size
CC=avr-gcc
CPP=avr-g++

INCLUDE=-Iavrtoolset
DEFINE=-DF_CPU=8000000UL -DATMEGA328P -DBAUD=$(BAUD)
CFLAGS=-Wall -Os -mmcu=$(DEVICE) -ffunction-sections -fdata-sections -flto
CPPFLAGS=$(CFLAGS) -w -std=gnu++11 -fno-threadsafe-statics
LDFLAGS=-flto -mmcu=$(DEVICE)
ADFLAGS=-p$(DEVICE) -c$(PROGRAMMER) -P$(PORT) -b$(BAUD) -v

all: snes_rx.hex

# fuse calc: https://www.engbedded.com/fusecalc/
writefuses:
	$(AVRDD) $(ADFLAGS) -U lfuse:w:0xe2:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m

dumpeeprom:
	$(AVRDD) $(ADFLAGS) -U eeprom:r:eeprom.dump:r

avrterm:
	$(AVRDD) $(ADFLAGS) -nt

install: snes_rx.hex
	$(AVRDD) $(ADFLAGS) -Uflash:w:$<:i

%.hex: %.elf
	$(AVRCP) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load \
		--no-change-warnings --change-section-lma .eeprom=0 $< $<.eep
	$(AVRCP) -O ihex -R .eeprom $< $@
	$(AVRSZ) --format=avr --mcu=$(DEVICE) $<

%.elf: %.o
	$(CC) $(LDFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(INCLUDE) $(DEFINE) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CPP) $(INCLUDE) $(DEFINE) $(CPPFLAGS) -c $< -o $@

clean:
	rm -rf `find . -name '*.o' -o -name '*.elf' -o -name '*.hex' \
		-o -name '*.eep' -o -name '*.d'`

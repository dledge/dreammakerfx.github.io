rm -R temp
mkdir temp
mkdir temp/dreammaker_fx_1.1.0

cp -R ~/Documents/Arduino/hardware/dreammaker_fx/samd/* temp/dreammaker_fx_1.1.0/

mkdir temp/dreammaker_fx_1.1.0/libraries/dmfx
cp -R ~/Documents/Arduino/libraries/dmfx-arduino/* temp/dreammaker_fx_1.1.0/libraries/dmfx/

mkdir temp/dreammaker_fx_1.1.0/examples
mkdir temp/dreammaker_fx_1.1.0/examples/dmfx_firmware_updater
cp -R ~/Github/Dreammaker_FX/dmfx-firmware-updater/dmfx-firmware-updater/* temp/dreammaker_fx_1.1.0/examples/dmfx_firmware_updater/


rm arduino-board-index/boards/dreammaker_fx_1.1.0.tar.bz2
tar -cvjSf arduino-board-index/boards/dreammaker_fx_1.1.0.tar.bz2 ./temp/dreammaker_fx_1.1.0/*

echo "SHA256:"
shasum -a 256 arduino-board-index/boards/dreammaker_fx_1.1.0.tar.bz2

echo "SIZE:"
stat -x arduino-board-index/boards/dreammaker_fx_1.1.0.tar.bz2
# 
# 
# 
rm -R temp
# satcom4pluto device type for OpenWebRX / OpenWebRX+
#
# A Pluto (clone) running the satcom4pluto satellite-downlink channelizer: the FPGA + the detector on
# the Pluto send only the detected downlinks; the SoapySDR module "satcom4pluto"
# (libsatcom4plutoSupport.so) rebuilds the full 240-270 MHz IQ stream (undetected parts are filled
# with noise shaped like the measured spectrum) and hands it to OpenWebRX like any wideband SDR.
#
# Install: copy this file to <openwebrx>/owrx/source/satcom4pluto.py (in Docker:
# /opt/openwebrx/owrx/source/satcom4pluto.py) and libsatcom4plutoSupport.so to the SoapySDR module
# folder, restart OpenWebRX. The type then appears as "satcom4pluto (Pluto satellite-downlink
# channelizer)" in Settings -> SDR devices and profiles.
from owrx.source.soapy import SoapyConnectorSource, SoapyConnectorDeviceDescription
from owrx.form.input import Input, TextInput
from owrx.form.input.validator import Range
from owrx.feature import FeatureDetector
from typing import List

# OpenWebRX only offers device types whose "feature" is known and available: register ours
# (the feature check runs "soapy_connector --listdrivers" and looks for "satcom4pluto").
if "satcom4pluto" not in FeatureDetector.features:
    FeatureDetector.features["satcom4pluto"] = ["soapy_connector", "soapy_satcom4pluto"]
if not hasattr(FeatureDetector, "has_soapy_satcom4pluto"):
    def _has_soapy_satcom4pluto(self):
        """
        The satcom4pluto SoapySDR module (libsatcom4plutoSupport.so) is required for a Pluto running the
        satcom4pluto satellite-downlink channelizer.
        """
        return self._has_soapy_driver("satcom4pluto")

    FeatureDetector.has_soapy_satcom4pluto = _has_soapy_satcom4pluto


class Satcom4PlutoSource(SoapyConnectorSource):
    def getDriver(self):
        return "satcom4pluto"

    def getEventNames(self):
        return super().getEventNames() + ["satcom4pluto_host"]

    def getSoapySettingsMappings(self):
        mappings = super().getSoapySettingsMappings()
        # detector options go to the Pluto at start and live whenever they are changed
        mappings.update({"satcom4pluto_detector": "sgchd"})
        return mappings

    def buildSoapyDeviceParameters(self, parsed, values):
        params = super().buildSoapyDeviceParameters(parsed, values)
        host = values["satcom4pluto_host"] if "satcom4pluto_host" in values and values["satcom4pluto_host"] else "192.168.5.5"
        params = [p for p in params if "uri" not in p]
        params += [{"uri": "ip:" + host}]
        return params


class Satcom4PlutoDeviceDescription(SoapyConnectorDeviceDescription):
    def getName(self):
        return "satcom4pluto (Pluto satellite-downlink channelizer)"

    def getInputs(self) -> List[Input]:
        return super().getInputs() + [
            TextInput(
                "satcom4pluto_host",
                "Pluto address",
                infotext="IP address of the Pluto on its USB network (default 192.168.5.5)",
            ),
            TextInput(
                "satcom4pluto_detector",
                "Detector options",
                infotext="Sent to the Pluto's detector, e.g. "
                + "-f 240:270 --max-snr-db 40 --kernel sin3 "
                + "(-f band in MHz; --max-snr-db: block signals more than this many dB above the noise, "
                + "e.g. local jammers; --spur MHZ: ignore a known birdie, repeatable up to 32; --pass LO:HI: always send this range, "
                + "e.g. a block of 5 kHz channels, repeatable; --max-bw-khz K: widest single detection, default 150). "
                + "No commas.",
            ),
        ]

    def getDeviceOptionalKeys(self):
        return super().getDeviceOptionalKeys() + ["satcom4pluto_host", "satcom4pluto_detector"]

    def getSampleRateRanges(self) -> List[Range]:
        return [Range(2500000, 61440000)]

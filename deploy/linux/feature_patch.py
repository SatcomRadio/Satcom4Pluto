
# --- satcom4pluto begin (added by install_satcom4pluto.sh; safe to remove together with satcom4pluto.py) ---
FeatureDetector.features.setdefault("satcom4pluto", ["soapy_connector", "soapy_satcom4pluto"])


def _satcom4pluto_has_soapy_satcom4pluto(self):
    """
    The satcom4pluto SoapySDR module (libsatcom4plutoSupport.so) is required for a Pluto running the
    satcom4pluto satellite-downlink channelizer.
    """
    return self._has_soapy_driver("satcom4pluto")


if not hasattr(FeatureDetector, "has_soapy_satcom4pluto"):
    FeatureDetector.has_soapy_satcom4pluto = _satcom4pluto_has_soapy_satcom4pluto
# --- satcom4pluto end ---

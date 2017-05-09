int UTMUPS::StandardZone(real lat, real lon, int setzone) {
    if (!(setzone >= MINPSEUDOZONE && setzone <= MAXZONE))
      throw GeographicErr("Illegal zone requested " + Utility::str(setzone));
    if (setzone >= MINZONE || setzone == INVALID)
      return setzone;
    if (Math::isnan(lat) || Math::isnan(lon)) // Check if lat or lon is a NaN
      return INVALID;
    if (setzone == UTM || (lat >= -80 && lat < 84)) {
      int ilon = int(floor(fmod(lon, real(360))));
      if (ilon >= 180)
        ilon -= 360;
      else if (ilon < -180)
        ilon += 360;
      int zone = (ilon + 186)/6;
      int band = MGRS::LatitudeBand(lat);
      if (band == 7 && zone == 31 && ilon >= 3)
        zone = 32;
      else if (band == 9 && ilon >= 0 && ilon < 42)
        zone = 2 * ((ilon + 183)/12) + 1;
      return zone;
    } else
      return UPS;
  }
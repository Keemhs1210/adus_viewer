#include "utm_transfer.h"

utm_transfer::utm_transfer(/* args */)
{
}

utm_transfer::~utm_transfer()
{
}


FLOAT utm_transfer::DegToRad(FLOAT deg) {
  	return (deg / 180.0 * pi);
}

FLOAT utm_transfer::RadToDeg(FLOAT rad) {
	return (rad / pi * 180.0);
}

FLOAT utm_transfer::ArcLengthOfMeridian (FLOAT phi) {
	FLOAT alpha, beta, gamma, delta, epsilon, n;
	FLOAT result;
	
	n = (sm_a - sm_b) / (sm_a + sm_b);
	
	alpha = ((sm_a + sm_b) / 2.0)
			* (1.0 + (POW(n, 2.0) / 4.0) + (POW(n, 4.0) / 64.0));
	
	beta = (-3.0 * n / 2.0) + (9.0 * POW(n, 3.0) / 16.0)
		+ (-3.0 * POW(n, 5.0) / 32.0);
	
	gamma = (15.0 * POW(n, 2.0) / 16.0)
			+ (-15.0 * POW(n, 4.0) / 32.0);
	
	delta = (-35.0 * POW(n, 3.0) / 48.0)
		+ (105.0 * POW(n, 5.0) / 256.0);
	
	epsilon = (315.0 * POW(n, 4.0) / 512.0);
	
	result = alpha
			* (phi + (beta * SIN(2.0 * phi))
			+ (gamma * SIN(4.0 * phi))
			+ (delta * SIN(6.0 * phi))
			+ (epsilon * SIN(8.0 * phi)));
	
	return result;
}

FLOAT utm_transfer::UTMCentralMeridian(int zone) {
	FLOAT cmeridian;
	cmeridian = DegToRad(-183.0 + ((FLOAT)zone * 6.0));
	
	return cmeridian;
}

FLOAT utm_transfer::FootpointLatitude(FLOAT y) {
	FLOAT y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
	FLOAT result;
	
	n = (sm_a - sm_b) / (sm_a + sm_b);
		
	alpha_ = ((sm_a + sm_b) / 2.0)
			* (1 + (POW(n, 2.0) / 4) + (POW(n, 4.0) / 64));
	
	y_ = y / alpha_;
	
	beta_ = (3.0 * n / 2.0) + (-27.0 * POW(n, 3.0) / 32.0)
			+ (269.0 * POW(n, 5.0) / 512.0);
	
	gamma_ = (21.0 * POW(n, 2.0) / 16.0)
			+ (-55.0 * POW(n, 4.0) / 32.0);
		
	delta_ = (151.0 * POW(n, 3.0) / 96.0)
			+ (-417.0 * POW(n, 5.0) / 128.0);
		
	epsilon_ = (1097.0 * POW(n, 4.0) / 512.0);
		
	result = y_ + (beta_ * SIN(2.0 * y_))
			+ (gamma_ * SIN(4.0 * y_))
			+ (delta_ * SIN(6.0 * y_))
			+ (epsilon_ * SIN(8.0 * y_));
	
	return result;
}

void utm_transfer::MapLatLonToXY (FLOAT phi, FLOAT lambda, FLOAT lambda0, FLOAT &x, FLOAT &y) {
    FLOAT N, nu2, ep2, t, t2, l;
    FLOAT l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;

    ep2 = (POW(sm_a, 2.0) - POW(sm_b, 2.0)) / POW(sm_b, 2.0);

    nu2 = ep2 * POW(COS(phi), 2.0);

    N = POW(sm_a, 2.0) / (sm_b * SQRT(1 + nu2));

    t = TAN(phi);
    t2 = t * t;
    
    l = lambda - lambda0;

    l3coef = 1.0 - t2 + nu2;

    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2
        - 58.0 * t2 * nu2;

    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2
        - 330.0 * t2 * nu2;

    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    x = N * COS(phi) * l
        + (N / 6.0 * POW(COS(phi), 3.0) * l3coef * POW(l, 3.0))
        + (N / 120.0 * POW(COS(phi), 5.0) * l5coef * POW(l, 5.0))
        + (N / 5040.0 * POW(COS(phi), 7.0) * l7coef * POW(l, 7.0));

    y = ArcLengthOfMeridian (phi)
        + (t / 2.0 * N * POW(COS(phi), 2.0) * POW(l, 2.0))
        + (t / 24.0 * N * POW(COS(phi), 4.0) * l4coef * POW(l, 4.0))
        + (t / 720.0 * N * POW(COS(phi), 6.0) * l6coef * POW(l, 6.0))
        + (t / 40320.0 * N * POW(COS(phi), 8.0) * l8coef * POW(l, 8.0));

    return;
}

void utm_transfer::MapXYToLatLon (FLOAT x, FLOAT y, FLOAT lambda0, FLOAT& phi, FLOAT& lambda)
{
	FLOAT phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
	FLOAT x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
	FLOAT x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;

	phif = FootpointLatitude (y);
		
	ep2 = (POW(sm_a, 2.0) - POW(sm_b, 2.0))
		/ POW(sm_b, 2.0);

	cf = COS(phif);
		
	nuf2 = ep2 * POW(cf, 2.0);
		
	Nf = POW(sm_a, 2.0) / (sm_b * SQRT(1 + nuf2));
	Nfpow = Nf;
		
	tf = TAN(phif);
	tf2 = tf * tf;
	tf4 = tf2 * tf2;

	x1frac = 1.0 / (Nfpow * cf);

	Nfpow *= Nf;   /* now equals Nf**2) */
	x2frac = tf / (2.0 * Nfpow);

	Nfpow *= Nf;   /* now equals Nf**3) */
	x3frac = 1.0 / (6.0 * Nfpow * cf);

	Nfpow *= Nf;   /* now equals Nf**4) */
	x4frac = tf / (24.0 * Nfpow);

	Nfpow *= Nf;   /* now equals Nf**5) */
	x5frac = 1.0 / (120.0 * Nfpow * cf);

	Nfpow *= Nf;   /* now equals Nf**6) */
	x6frac = tf / (720.0 * Nfpow);

	Nfpow *= Nf;   /* now equals Nf**7) */
	x7frac = 1.0 / (5040.0 * Nfpow * cf);

	Nfpow *= Nf;   /* now equals Nf**8) */
	x8frac = tf / (40320.0 * Nfpow);

	x2poly = -1.0 - nuf2;

	x3poly = -1.0 - 2 * tf2 - nuf2;

	x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2
			- 3.0 * (nuf2 *nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

	x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

	x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2
			+ 162.0 * tf2 * nuf2;

	x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

	x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);
		
	phi = phif + x2frac * x2poly * (x * x)
		+ x4frac * x4poly * POW(x, 4.0)
		+ x6frac * x6poly * POW(x, 6.0)
		+ x8frac * x8poly * POW(x, 8.0);
		
	lambda = lambda0 + x1frac * x
			+ x3frac * x3poly * POW(x, 3.0)
			+ x5frac * x5poly * POW(x, 5.0)
			+ x7frac * x7poly * POW(x, 7.0);
		
	return;
}

int utm_transfer::LatLonToUTMXY (FLOAT lat, FLOAT lon, int zone, FLOAT& x, FLOAT& y) {
	if ( (zone < 1) || (zone > 60) )
		zone = FLOOR((lon + 180.0) / 6) + 1;
	
	MapLatLonToXY (DegToRad(lat), DegToRad(lon), UTMCentralMeridian(zone), x, y);
	
	x = x * UTMScaleFactor + 500000.0;
	y = y * UTMScaleFactor;
	if (y < 0.0)
		y = y + 10000000.0;
	
	return zone;
}

void utm_transfer::UTMXYToLatLon (FLOAT x, FLOAT y, int zone, bool southhemi, FLOAT& lat, FLOAT& lon) {
	FLOAT cmeridian;
		
	x -= 500000.0;
	x /= UTMScaleFactor;
		
	if (southhemi)
		y -= 10000000.0;
		
	y /= UTMScaleFactor;
	
	cmeridian = UTMCentralMeridian (zone);
	MapXYToLatLon (x, y, cmeridian, lat, lon);
		
	return;
}
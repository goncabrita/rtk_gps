#ifndef CONV_H
#define CONV_H

#include <math.h>


#define WSG84_A 6378137.0
#define E2 0.0066943799831668
#define Ee2  0.0067394967352076

void GPStoUTM(const double lat, const double longi, double &northing, double &easting, int &n_zona, char &letra)
{
	

	if     ((84 >= lat) && (lat >= 72))  letra = 'X';
	else if ((72 > lat) && (lat >= 64))  letra = 'W';
	else if ((64 > lat) && (lat >= 56))  letra = 'V';
	else if ((56 > lat) && (lat >= 48))  letra = 'U';
	else if ((48 > lat) && (lat >= 40))  letra = 'T';
	else if ((40 > lat) && (lat >= 32))  letra = 'S';
	else if ((32 > lat) && (lat >= 24))  letra = 'R';
	else if ((24 > lat) && (lat >= 16))  letra = 'Q';
	else if ((16 > lat) && (lat >= 8))   letra = 'P';
	else if (( 8 > lat) && (lat >= 0))   letra = 'N';
	else if (( 0 > lat) && (lat >= -8))  letra = 'M';
	else if ((-8 > lat) && (lat >= -16)) letra = 'L';
	else if((-16 > lat) && (lat >= -24)) letra = 'K';
	else if((-24 > lat) && (lat >= -32)) letra = 'J';
	else if((-32 > lat) && (lat >= -40)) letra = 'H';
	else if((-40 > lat) && (lat >= -48)) letra = 'G';
	else if((-48 > lat) && (lat >= -56)) letra = 'F';
	else if((-56 > lat) && (lat >= -64)) letra = 'E';
	else if((-64 > lat) && (lat >= -72)) letra = 'D';
	else if((-72 > lat) && (lat >= -80)) letra = 'C';     
	else letra = '-';
	
	n_zona = int((longi + 180)/6) + 1;
	
	//caso especial na noruega
	if( lat >= 56.0 && lat < 64.0 && longi >= 3.0 && longi < 12.0 )
		n_zona = 32;

        // caso especial de Svalbard
	if( lat >= 72.0 && lat < 84.0 )
	{
	  if(      longi >= 0.0  && longi <  9.0 ) n_zona = 31;
	  else if( longi >= 9.0  && longi < 21.0 ) n_zona = 33;
	  else if( longi >= 21.0 && longi < 33.0 ) n_zona = 35;
	  else if( longi >= 33.0 && longi < 42.0 ) n_zona = 37;
	}
	
	double lat_rad = lat * M_PI / 180.0;
	double longi_rad = longi * M_PI / 180;
	
	double origem_longi = (n_zona - 1)*6 - 180 + 3;
	double origem_longi_rad = origem_longi * M_PI / 180.0;
	
	double a = WSG84_A;
	double e_sq = E2;
	double e_sq2 = Ee2; // e'²
	double N = a / sqrt ( 1 - e_sq * (sin(lat_rad)*sin(lat_rad)));//N = a/sqrt(1-e_sq*(sin(lat_rad)^2));
	double T = tan(lat_rad)*tan(lat_rad); //T = tan(lat_rad)^2;
	double C = e_sq2*cos(lat_rad)*cos(lat_rad);
	double A = cos(lat_rad)*(longi_rad - origem_longi_rad);
	
	double M = a*((1-e_sq/4 - 3*e_sq*e_sq/64 - 5*e_sq*e_sq*e_sq)*lat_rad - 
    		(3*e_sq/8 + 3*e_sq*e_sq/32 + 45*e_sq*e_sq*e_sq/1024)*sin(2*lat_rad) +
    		(15*e_sq*e_sq/256 + 45*e_sq*e_sq*e_sq/1024) *sin(4*lat_rad) + 
    		(35*e_sq*e_sq*e_sq/3072)*sin(6*lat_rad));
    		
    	double k0 = 0.9996;
    	
    	easting = k0*N*(A + (1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C - 58*e_sq2)*A*A*A*A*A/120) + 500000;
    	
    	northing = k0*(M+ N*tan(lat_rad)*(A*A/2 + (5 - T + 9*C + 4*C*C)*A*A*A*A/24 + 
    (61-58*T+T*T + 600*C - 330*e_sq2)*A*A*A*A*A*A/720));
	
}	
#endif
	

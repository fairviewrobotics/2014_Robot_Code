#include <stdio.h>
#include <math.h>

double CalculateFromTurn(double Ri, double Rf, double Li, double Lf, double r, double angle)
{
	double L = Lf - Li;
	double R = Rf - Ri;
	double Langle = -L/r;
	double Rangle = R/r;
	double Nangle = (Langle + Rangle)/2;
	return Nangle+angle;
}

<<<<<<< HEAD
{
}
=======
>>>>>>> Autonomous_1
void CalculateFromStraight(double Rf, double Ri, double Lf, double Li, double angle, double x, double y)
{
	double dl = Lf - Li;
	double dr = Rf - Ri;
	double d = (dl+dr)/2;
	double x1 = x + d * sin(angle);
	double y1 = y + d * cos(angle);
	printf ("(%f, %f)\n", x1, y1);
}

int main()
{
	CalculateFromStraight(20,10,21,11,0.5235987, 0, 0);
	return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> Autonomous_1

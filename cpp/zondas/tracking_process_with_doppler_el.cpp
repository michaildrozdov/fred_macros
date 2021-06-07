

#include "externs.h"

//globals from macro
float x = 0;
float y = 0;
float z = 0;
int lastIndex = 0;
MatrixSimple xprev(1, 1);
MatrixSimple P(1, 1);
MatrixSimple A(1, 1);
MatrixSimple Xpmat(1, 1);
MatrixSimple Pp(1, 1);
MatrixSimple meas(1, 1);
MatrixSimple innov(1, 1);
MatrixSimple S(1,1);
MatrixSimple innovationError(1,1);
float additional = 0;
MatrixSimple K(1, 1);
MatrixSimple Xn(1, 1);
MatrixSimple xxTemp(1, 1);
MatrixSimple ppTemp(1, 1);
float scaledError = 0;
float range = 0;
float range2 = 0;
float az = 0;
float el = 0;

static float xtrack = 0;
static float ytrack = 0;
static float ztrack = 0;

float az2 = 0;
float el2 = 0;

float xtrack2 = 0;
float ytrack2 = 0;
float ztrack2 = 0;

float dpos = 0;
float velRatio = 0;
int tr2 = 0;
int tr = 0;

int tracking_process_with_doppler_el(
	FloatSignal& timestamps,
	MatrixSimple& states, MatrixSimple& covariances, FloatSignal& thresholds, FloatSignal& ages,
	int& numOfTracks, int& maxTracks, FloatSignal& rData, FloatSignal& aData, FloatSignal& eData,
	FloatSignal& vData, FloatSignal& isUsedData, FloatSignal& isMatchedData,
	FloatSignal& idsData, int& numOfData, float& currentTime,FloatSignal& idsTracks,int& lastTrackId
)
{
	//out(numOfTracks);
	FloatSignal timestampso = timestamps;

	//list of gated tracks
	int maxNumGated = 20;

	int numGated = 0;

	//macro line
	/*rDataGated = signal(1, maxNumGated, false);//array of distances
	aDataGated = signal(1, maxNumGated, false);//array of az angles
	eDataGated = signal(1, maxNumGated, false);//array of el angles
	vDataGated = signal(1, maxNumGated, false);//array of velocities*/
	//converted to
	FloatSignal rDataGated(1, maxNumGated);//array of distances
	FloatSignal aDataGated(1, maxNumGated);//array of az angles
	FloatSignal eDataGated(1, maxNumGated);//array of el angles
	FloatSignal vDataGated(1, maxNumGated);//array of velocities*/
	//end

	//macro line
	/*isUsedDataGated = signal(1, maxNumGated, false);
	isMatchedDataGated = signal(1, maxNumGated, false);
	idsDataGated = signal(1, maxNumGated, false);*/
	//converted to
	FloatSignal isUsedDataGated(1, maxNumGated);
	FloatSignal isMatchedDataGated(1, maxNumGated);
	FloatSignal idsDataGated(1, maxNumGated); 
	//end

	int numOtherMatches = 0;

	//macros line
	/*rDataOtherMatches = signal(1, maxNumGated, false);//array of distances
	aDataOtherMatches = signal(1, maxNumGated, false);//array of angles
	eDataOtherMatches = signal(1, maxNumGated, false);//array of angles
	vDataOtherMatches = signal(1, maxNumGated, false);//array of velocities
	isUsedDataOtherMatches = signal(1, maxNumGated, false);
	isMatchedDataOtherMatches = signal(1, maxNumGated, false);
	idsDataOtherMatches = signal(1, maxNumGated, false);*/
	//converted to
	FloatSignal rDataOtherMatches(1, maxNumGated);//array of distances
	FloatSignal aDataOtherMatches(1, maxNumGated);//array of angles
	FloatSignal eDataOtherMatches(1, maxNumGated);//array of angles
	FloatSignal vDataOtherMatches(1, maxNumGated);//array of velocities
	FloatSignal isUsedDataOtherMatches(1, maxNumGated);
	FloatSignal isMatchedDataOtherMatches(1, maxNumGated);
	FloatSignal idsDataOtherMatches(1, maxNumGated);
	//end

	//tracking parameters
	float dt = 1;//frames
	float maxVel = 10;//m/frame
	float minVel = 0.1;//m/s
	float kMatches = 1;//how many updated Kalman states can be created from single one with measurements close enough to estimation
	float trueTrackThreshold = 1.5;//match factor defining, what is considered true track, less is for less false tracks
	float removeTrackThreshold = 10;//factor defining, threshold for track removal, less is for faster tracks removal
	float minAgeToDisplayRadial = 4;//track with mostly radial movement needs to be updated for this amount of frames to be displayed
	float minAgeToDisplayTangental = 5;//track with mostly tangental movement needs to be updated for this amount of frames to be displayed

	float matchingVelocityThreshold = 0.5;//0.8 means within 80%
	float matchingPositionThreshold = 1;

	//R = 5*eye(3);//sensor noise for r, a, v and vy, should optimize this, smaller number means believing measurement more than model
	//R(2,2) = 10;
	MatrixSimple R(4, 4);
	R = 0;
	set_matrix(R, 0, 0, 0.3);
	set_matrix(R, 1, 1, 0.3);
	set_matrix(R, 2, 2, 10); //az angle estimation is very noisy
	set_matrix(R, 3, 3, 20); //el angle estimation is most noisy
	//float rr11 = get_matrix(R, 0, 0); //value against which the track distance covariance is compared
	//float rr33 = get_matrix(R, 2, 2);//value against which the track angle covariance is compared
	//float rr44 = get_matrix(R, 3, 3);//value against which the track angle covariance is compared

	//Q = 0.05*eye(6);//process noise for Kalman filter estimation, smaller number means believing model more than measurement
	MatrixSimple Q(6, 6);
	Q = 0;
	set_matrix(Q, 0, 0, 0.1f);
	set_matrix(Q, 1, 1, 0.1f);
	set_matrix(Q, 2, 2, 5);
	set_matrix(Q, 3, 3, 1);
	set_matrix(Q, 4, 4, 10);
	set_matrix(Q, 5, 5, 1);

	//Pinit = 5*eye(6);//initial covariance matrix for track, smaller means we believe first measurement more
	//Pinit(1,1) = 2;//smaller value for distance
	MatrixSimple Pinit(6, 6);
	Pinit = 0;
	set_matrix(Pinit, 0, 0, 2);
	set_matrix(Pinit, 1, 1, 2);
	set_matrix(Pinit, 2, 2, 10);
	set_matrix(Pinit, 3, 3, 10);
	set_matrix(Pinit, 4, 4, 20);
	set_matrix(Pinit, 5, 5, 10);
	float rr11 = 0.5f * get_matrix(Pinit, 0, 0); //value against which the track distance covariance is compared
	float rr33 = get_matrix(Pinit, 2, 2);//value against which the track angle covariance is compared
	float rr44 = get_matrix(Pinit, 4, 4);//value against which the track angle covariance is compared

	//H is 3x4 observation matrix, since three variables are observed
	MatrixSimple H(4, 6);
	H = 0;
	set_matrix(H, 0, 0, 1);
	set_matrix(H, 1, 1, 1);
	set_matrix(H, 2, 2, 1);
	set_matrix(H, 3, 4, 1);

	float sqMaxVel = maxVel * maxVel;//square of velocity for position comparison
	float sqMinVel = minVel * minVel;//square of velocity for position comparison 

	//print_line("In processing number of tracks ", numOfTracks);
	for (int tr = 0; tr < numOfTracks; tr = tr + 1)
	{
		//1) crude gate (by max velocity)
		numGated = 0;
		//currentTrack = currentTracks(tr);
		range = get_matrix(states, tr, 0);
		float angle = get_matrix(states, tr, 2);
		float elevation = get_matrix(states, tr, 4);
		//print_line("Examining track ",  range, " ",  x21[tr], " ", angle, " ",  x41[tr]);

		float xtrack = range * sin(angle * pi() / 180) * cos(elevation * pi() / 180); //along az
		float ytrack = range * cos(angle * pi() / 180) * sin(elevation * pi() / 180); //along el
		float ztrack = range * cos(angle * pi() / 180) * cos(elevation * pi() / 180); //from the radar

		for (int dp = 0; dp < numOfData; dp = dp + 1)
		{
			 x = rData.float_data[dp] * sin(aData.float_data[dp] * pi() / 180) * cos(eData.float_data[dp] * pi() / 180);
			 y = rData.float_data[dp] * cos(aData.float_data[dp] * pi() / 180) * sin(eData.float_data[dp] * pi() / 180);
			 z = rData.float_data[dp] * cos(aData.float_data[dp] * pi() / 180) * cos(eData.float_data[dp] * pi() / 180);

			float distanceSq = (xtrack - x)*(xtrack - x) + (ytrack - y)*(ytrack - y) + (ztrack - z)*(ztrack - z);
			//An additional requirement of small displacement along the z dimension
			if ((distanceSq < sqMaxVel) && ((ztrack - z)*(ztrack - z) < sqMaxVel / 200))
			{
				if (numGated < maxNumGated)
				{
					rDataGated.float_data[numGated] = rData.float_data[dp];
					aDataGated.float_data[numGated] = aData.float_data[dp];
					vDataGated.float_data[numGated] = vData.float_data[dp];
					eDataGated.float_data[numGated] = eData.float_data[dp];
					isUsedDataGated.float_data[numGated] = isUsedData.float_data[dp];
					isMatchedDataGated.float_data[numGated] = isMatchedData.float_data[dp];
					idsDataGated.float_data[numGated] = idsData.float_data[dp];

					numGated = numGated + 1;
				}
			}
		}
		//print_line("Gated measurments found ", numGated);


		//gatedMeasurements = length(gated);

		//2) find best matching update
		numOtherMatches = 0;


		//cluster close points
		int g = 0;//1 in Octave
		while (g < numGated)
		{
			x = rDataGated.float_data[g] * sin(aDataGated.float_data[g] * pi() / 180) * cos(eDataGated.float_data[g] * pi() / 180);
			y = rDataGated.float_data[g] * cos(aDataGated.float_data[g] * pi() / 180) * sin(eDataGated.float_data[g] * pi() / 180);
			z = rDataGated.float_data[g] * cos(aDataGated.float_data[g] * pi() / 180) * cos(eDataGated.float_data[g] * pi() / 180);

			int gg = g + 1;
			while (gg < numGated)
			{
				float x2 = rDataGated.float_data[gg] * sin(aDataGated.float_data[gg] * pi() / 180) * cos(eDataGated.float_data[gg] * pi() / 180);
				float y2 = rDataGated.float_data[gg] * cos(aDataGated.float_data[gg] * pi() / 180) * sin(eDataGated.float_data[gg] * pi() / 180);
				float z2 = rDataGated.float_data[gg] * cos(aDataGated.float_data[gg] * pi() / 180) * cos(eDataGated.float_data[gg] * pi() / 180);

				float dposSq = (x2 - x)*(x2 - x) + (y2 - y)*(y2 - y) + (z2 - z)*(z2 - z);
				float velRatio = vDataGated.float_data[g] / (vDataGated.float_data[gg] + 0.1);
				bool velMatches = (velRatio > 0) || ((abs(vDataGated.float_data[g]) < 0.2) && (abs(vDataGated.float_data[gg]) < 0.2));
				if ((dposSq < matchingPositionThreshold*matchingPositionThreshold) && velMatches)
				{
					rDataGated.float_data[g] = (rDataGated.float_data[g] + rDataGated.float_data[gg]) / 2;
					aDataGated.float_data[g] = (aDataGated.float_data[g] + aDataGated.float_data[gg]) / 2;
					vDataGated.float_data[g] = (vDataGated.float_data[g] + vDataGated.float_data[gg]) / 2;
					eDataGated.float_data[g] = (eDataGated.float_data[g] + eDataGated.float_data[gg]) / 2;

					if (gg + 1 > numGated)
					{
						lastIndex = numGated - 1;
						rDataGated.float_data[gg] = rDataGated.float_data[lastIndex];
						aDataGated.float_data[gg] = aDataGated.float_data[lastIndex];
						vDataGated.float_data[gg] = vDataGated.float_data[lastIndex];
						eDataGated.float_data[gg] = eDataGated.float_data[lastIndex];
						isUsedDataGated.float_data[gg] = isUsedDataGated.float_data[lastIndex];
						isMatchedDataGated.float_data[gg] = isMatchedDataGated.float_data[lastIndex];
						idsDataGated.float_data[gg] = idsDataGated.float_data[lastIndex];

					}
					numGated = numGated - 1;
				}
				else
				{
					gg = gg + 1;
				}
			}
			g = g + 1;
		}
		//print_line("close points clustered");

		xprev = MatrixSimple(6, 1);
		for (int xpos = 0; xpos < 6; xpos = xpos + 1)
		{
			set_matrix(xprev, xpos, 0, get_matrix(states, tr, xpos));
		}

		//previous covariance
		P = MatrixSimple(6, 6);
		for (int ppos = 0; ppos < 6; ppos = ppos + 1)
		{
			set_matrix_row(P, ppos, get_matrix_row(covariances, tr * 6 + ppos));
		}

		dt = currentTime - timestamps.float_data[tr];
		//print_line("dt = ", dt);
		//process matrix
		A = MatrixSimple(6, 6);
		set_matrix(A, 0, 0, 1);
		set_matrix(A, 0, 1, dt);
		set_matrix(A, 0, 2, 0);
		set_matrix(A, 0, 3, 0);
		set_matrix(A, 0, 4, 0);
		set_matrix(A, 0, 5, 0);

		set_matrix(A, 1, 0, 0);
		set_matrix(A, 1, 1, 1);
		set_matrix(A, 1, 2, 0);
		set_matrix(A, 1, 3, 0);
		set_matrix(A, 1, 4, 0);
		set_matrix(A, 1, 5, 0);

		set_matrix(A, 2, 0, 0);
		set_matrix(A, 2, 1, 0);
		set_matrix(A, 2, 2, 1);
		set_matrix(A, 2, 3, dt);
		set_matrix(A, 2, 4, 0);
		set_matrix(A, 2, 5, 0);

		set_matrix(A, 3, 0, 0);
		set_matrix(A, 3, 1, 0);
		set_matrix(A, 3, 2, 0);
		set_matrix(A, 3, 3, 1);
		set_matrix(A, 3, 4, 0);
		set_matrix(A, 3, 5, 0);

		set_matrix(A, 4, 0, 0);
		set_matrix(A, 4, 1, 0);
		set_matrix(A, 4, 2, 0);
		set_matrix(A, 4, 3, 0);
		set_matrix(A, 4, 4, 1);
		set_matrix(A, 4, 5, dt);

		set_matrix(A, 5, 0, 0);
		set_matrix(A, 5, 1, 0);
		set_matrix(A, 5, 2, 0);
		set_matrix(A, 5, 3, 0);
		set_matrix(A, 5, 4, 0);
		set_matrix(A, 5, 5, 1);

		//prior state Xp = A*X;
		Xpmat = A * xprev;
		//localx11 = get_matrix(Xpmat, 0, 0);
		//localx21 = get_matrix(Xpmat, 1, 0);
		//localx31 = get_matrix(Xpmat, 2, 0);
		//localx41 = get_matrix(Xpmat, 3, 0);
		//localx51 = get_matrix(Xpmat, 4, 0);
		//localx61 = get_matrix(Xpmat, 5, 0);
		

		//prior covarianc Pp = A*P*A'+ Q;
		Pp = A * P* MatrixSimple::transpose(A) + Q;
		//print_line("after prior covariance calc");
		//print_line("Prior state: ", localx11, ", ", localx21, ", ", localx31, ", ", localx41);
		//print_line("Pp=[", get_matrix(Pp,0, 0), " ", get_matrix(Pp,0, 1), " ", get_matrix(Pp,0, 2), " ", get_matrix(Pp,0, 3), " ", get_matrix(Pp,0, 4), " ", get_matrix(Pp,0, 5), ";");
		//print_line("       ", get_matrix(Pp,1, 0), " ", get_matrix(Pp,1, 1), " ", get_matrix(Pp,1, 2), " ", get_matrix(Pp,1, 3), " ", get_matrix(Pp,1, 4), " ", get_matrix(Pp,1, 5), ";");
		//print_line("       ", get_matrix(Pp,2, 0), " ", get_matrix(Pp,2, 1), " ", get_matrix(Pp,2, 2), " ", get_matrix(Pp,2, 3), " ", get_matrix(Pp,2, 4), " ", get_matrix(Pp,2, 5), ";");
		//print_line("       ", get_matrix(Pp,3, 0), " ", get_matrix(Pp,3, 1), " ", get_matrix(Pp,3, 2), " ", get_matrix(Pp,3, 3), " ", get_matrix(Pp,3, 4), " ", get_matrix(Pp,3, 5), ";");
		//print_line("       ", get_matrix(Pp,4, 0), " ", get_matrix(Pp,4, 1), " ", get_matrix(Pp,4, 2), " ", get_matrix(Pp,4, 3), " ", get_matrix(Pp,4, 4), " ", get_matrix(Pp,4, 5), ";");
		//print_line("       ", get_matrix(Pp,5, 0), " ", get_matrix(Pp,5, 1), " ", get_matrix(Pp,5, 2), " ", get_matrix(Pp,5, 3), " ", get_matrix(Pp,5, 4), " ", get_matrix(Pp,5, 5), "]");
		//print_line("after prior state calc");

		//The scaling below takes ratios of variances of range, az and el and apropriate fields of the measurement noise matrix
		float scaling = get_matrix(covariances, tr * 6, 0) / rr11 * get_matrix(covariances, tr * 6 + 2, 2) / rr33 * get_matrix(covariances, tr * 6 + 4, 4) / rr44;
		//print_line("Scaling parameter ", scaling);

		float bestThreshold = removeTrackThreshold;//any track with better matching should be above this
		bool updatedByMeas = false;
		for (g = 0; g < numGated; g = g + 1)
		{
			//measurement z is 3x1 vector
			meas = MatrixSimple(4, 1);
			set_matrix(meas, 0, 0, rDataGated.float_data[g]);
			set_matrix(meas, 1, 0, vDataGated.float_data[g]);
			set_matrix(meas, 2, 0, aDataGated.float_data[g]);
			set_matrix(meas, 3, 0, eDataGated.float_data[g]);
			//print_line("Updating with measurement ", rDataGated[g], ", ", vDataGated[g], ", ", aDataGated[g]);

			//Kalman filter update below

			//Innovation Inn=z-H*Xp
			//out(z, H, xprev);
			 innov = meas - H * Xpmat;
			//print_line("after innovation calc");
			//print_line("Innovation ", get_matrix(innov, 0,0), " ", get_matrix(innov, 1,0), " ", get_matrix(innov, 2,0));

			//measurement prediction covariance S = H*Pp*H'  + R
			S = H * Pp*MatrixSimple::transpose(H) + R;
			//print_line("after prediction covariance");
			//print_line("S=[", get_matrix(S,0, 0), " ", get_matrix(S,0, 1), " ", get_matrix(S,0, 2), " ", get_matrix(S,0, 3), ";");
			//print_line("       ", get_matrix(S,1, 0), " ", get_matrix(S,1, 1), " ", get_matrix(S,1, 2), " ", get_matrix(S,1, 3),  ";");
			//print_line("       ", get_matrix(S,2, 0), " ", get_matrix(S,2, 1), " ", get_matrix(S,2, 2), " ", get_matrix(S,2, 3),  ";");
			//print_line("       ", get_matrix(S,3, 0), " ", get_matrix(S,3, 1), " ", get_matrix(S,3, 2), " ", get_matrix(S,3, 3),  "]");

			innovationError = MatrixSimple::transpose(innov)*inverse_mkl(S)*innov;
			//print_line("Mahalanobis distance ", get_matrix(innovationError,0,0), ", best threshold ", bestThreshold);

			if (get_matrix(innovationError, 0, 0)*scaling < bestThreshold)//best update so far
			{
				updatedByMeas = true;
				//print_line("updating state");
				bestThreshold = scaling * get_matrix(innovationError, 0, 0);
				//Kalman gain K = Pp * H' * inv(S);
				K = Pp * MatrixSimple::transpose(H)*inverse_mkl(S);
				//print_line("after Kalman gain calc");

				//Xnew = Xp + K * Inn; //new state
				Xn = Xpmat + K * innov;
				//print_line("after state upate");
				//Pnew = Pp - K * H * Pp;//new covariance
				P = Pp - K * H*Pp;
				//print_line("after covariance update");

				xxTemp = Xn;
				ppTemp = P;

				//x11temp = get_matrix(Xn,0,0);
				//x21temp = get_matrix(Xn,1,0);
				//x31temp = get_matrix(Xn,2,0);
				//x41temp = get_matrix(Xn,3,0);
				//x51temp = get_matrix(Xn,4,0);
				//x61temp = get_matrix(Xn,5,0);

				//p11temp = get_matrix(P,0,0);
				//p12temp = get_matrix(P,0,1);
				//p13temp = get_matrix(P,0,2);
				//p14temp = get_matrix(P,0,3);
				//p15temp = get_matrix(P,0,4);
				//p16temp = get_matrix(P,0,5);

				//p21temp = get_matrix(P,1,0);
				//p22temp = get_matrix(P,1,1);
				//p23temp = get_matrix(P,1,2);
				//p24temp = get_matrix(P,1,3);
				//p25temp = get_matrix(P,1,4);
				//p26temp = get_matrix(P,1,5);

				//p31temp = get_matrix(P,2,0);
				//p32temp = get_matrix(P,2,1);
				//p33temp = get_matrix(P,2,2);
				//p34temp = get_matrix(P,2,3);
				//p35temp = get_matrix(P,2,4);
				//p36temp = get_matrix(P,2,5);

				//p41temp = get_matrix(P,3,0);
				//p42temp = get_matrix(P,3,1);
				//p43temp = get_matrix(P,3,2);
				//p44temp = get_matrix(P,3,3);
				//p45temp = get_matrix(P,3,4);
				//p46temp = get_matrix(P,3,5);

				//p51temp = get_matrix(P,4,0);
				//p52temp = get_matrix(P,4,1);
				//p53temp = get_matrix(P,4,2);
				//p54temp = get_matrix(P,4,3);
				//p55temp = get_matrix(P,4,4);
				//p56temp = get_matrix(P,4,5);

				//p61temp = get_matrix(P,5,0);
				//p62temp = get_matrix(P,5,1);
				//p63temp = get_matrix(P,5,2);
				//p64temp = get_matrix(P,5,3);
				//p65temp = get_matrix(P,5,4);
				//p66temp = get_matrix(P,5,5);

				//thresholdsemp = get_matrix(innovationError,0,0);
				//timestampsTemp = currentTime;
			}

			if (get_matrix(innovationError, 0, 0)*scaling < removeTrackThreshold)//update of track is vaid, not necessary best
			{
				//print_line("setting measurement as used");
				isMatchedDataGated.float_data[g] = 1;
				for (int dp = 0; dp < numOfData; dp = dp + 1)
				{
					if (idsData.float_data[dp] == idsDataGated.float_data[g])
					{
						isMatchedData.float_data[dp] = 1;
					}
				}
			}
		}
		timestamps.float_data[tr] = currentTime;
		if (updatedByMeas)
		{
			for (int xpos = 0; xpos < 6; xpos = xpos + 1)
			{
				set_matrix(states, tr, xpos, get_matrix(xxTemp, xpos, 0));
			}
			for (int ppos = 0; ppos < 6; ppos = ppos + 1)
			{
				set_matrix_row(covariances, tr * 6 + ppos, get_matrix_row(ppTemp, ppos));
			}
			//x11[tr] = x11temp;
			//x21[tr] = x21temp;
			//x31[tr] = x31temp;
			//x41[tr] = x41temp;

			//p11[tr] = p11temp;
			//p12[tr] = p12temp;
			//p13[tr] = p13temp;
			//p14[tr] = p13temp;

			//p21[tr] = p21temp;
			//p22[tr] = p22temp;
			//p23[tr] = p23temp;
			//p24[tr] = p24temp;

			//p31[tr] = p31temp;
			//p32[tr] = p32temp;
			//p33[tr] = p33temp;
			//p34[tr] = p34temp;

			//p41[tr] = p41temp;
			//p42[tr] = p42temp;
			//p43[tr] = p43temp;
			//p44[tr] = p44temp;

			thresholds.float_data[tr] = bestThreshold;
			ages.float_data[tr] = ages.float_data[tr] + 1;//dt here is 1
		}
		else
		{
			for (int xpos = 0; xpos < 6; xpos = xpos + 1)
			{
				set_matrix(states, tr, xpos, get_matrix(Xpmat, xpos, 0));
			}
			for (int ppos = 0; ppos < 6; ppos = ppos + 1)
			{
				set_matrix_row(covariances, tr * 6 + ppos, get_matrix_row(Pp, ppos));
			}
			//x11[tr] = get_matrix(Xpmat,0,0);
			//x21[tr] = get_matrix(Xpmat,1,0);
			//x31[tr] = get_matrix(Xpmat,2,0);
			//x41[tr] = get_matrix(Xpmat,3,0);

			//p11[tr] = get_matrix(Pp,0,0);
			//p12[tr] = get_matrix(Pp,0,1);
			//p13[tr] = get_matrix(Pp,0,2);
			//p14[tr] = get_matrix(Pp,0,3);

			//p21[tr] = get_matrix(Pp,1,0);
			//p22[tr] = get_matrix(Pp,1,1);
			//p23[tr] = get_matrix(Pp,1,2);
			//p24[tr] = get_matrix(Pp,1,3);

			//p31[tr] = get_matrix(Pp,2,0);
			//p32[tr] = get_matrix(Pp,2,1);
			//p33[tr] = get_matrix(Pp,2,2);
			//p34[tr] = get_matrix(Pp,2,3);

			//p41[tr] = get_matrix(Pp,3,0);
			//p42[tr] = get_matrix(Pp,3,1);
			//p43[tr] = get_matrix(Pp,3,2);
			//p44[tr] = get_matrix(Pp,3,3);

			if (1.4 > 1 + 0.8 / ages.float_data[tr])
			{
				thresholds.float_data[tr] = thresholds.float_data[tr] * 1.4 + 0.15;
			}
			else
			{
				thresholds.float_data[tr] = thresholds.float_data[tr] * (1 + 0.8 / ages.float_data[tr]) + 0.15;
			}
		}
		scaledError = thresholds.float_data[tr] * get_matrix(covariances, tr * 6, 0) / rr11 * get_matrix(covariances, tr * 6 + 2, 2) / rr33 * get_matrix(covariances, tr * 6 + 4, 4) / rr44;
		//scaledError = thresholds[tr]*p11[tr]/rr11*p33[tr]/rr33;
		//print_line("Updated values ", x11[tr], ", ", x21[tr], ", ", x31[tr], ", ", x41[tr]);
		//print_line("error=", thresholds[tr], ", covariances ", p11[tr], ", ", p33[tr], " scaled error=", scaledError);
	}
	//print_line("After update thresholds");
	//print_line("");


	//3) update tracks without detections
	tr = 0;
	while (tr < numOfTracks)
	{
		scaledError = thresholds.float_data[tr] * get_matrix(covariances, tr * 6, 0) / rr11 * get_matrix(covariances, tr * 6 + 2, 2) / rr33 * get_matrix(covariances, tr * 6 + 4, 4) / rr44;
		//scaledError = thresholds[tr]*p11[tr]/rr11*p33[tr]/rr33;
		if (scaledError > removeTrackThreshold)//should remove track
		{
			if (tr + 1 < numOfTracks)
			{
				lastIndex = numOfTracks - 1;
				//print_line("Substituting track ", tr, " with ", lastIndex);

				set_matrix_row(states, tr, get_matrix_row(states, lastIndex));

				for (int ppos = 0; ppos < 6; ppos = ppos + 1)
				{
					set_matrix_row(covariances, tr * 6 + ppos, get_matrix_row(covariances, lastIndex * 6 + ppos));
				}
				//x11[tr] = x11[lastIndex];
				//x21[tr] = x21[lastIndex];
				//x31[tr] = x31[lastIndex];
				//x41[tr] = x41[lastIndex];

				//p11[tr] = p11[lastIndex];
				//p12[tr] = p12[lastIndex];
				//p13[tr] = p13[lastIndex];
				//p14[tr] = p14[lastIndex];
				//p21[tr] = p21[lastIndex];
				//p22[tr] = p22[lastIndex];
				//p23[tr] = p23[lastIndex];
				//p24[tr] = p24[lastIndex];
				//p31[tr] = p31[lastIndex];
				//p32[tr] = p32[lastIndex];
				//p33[tr] = p33[lastIndex];
				//p34[tr] = p34[lastIndex];
				//p41[tr] = p41[lastIndex];
				//p42[tr] = p42[lastIndex];
				//p43[tr] = p43[lastIndex];
				//p44[tr] = p44[lastIndex];

				thresholds.float_data[tr] = thresholds.float_data[lastIndex];
				ages.float_data[tr] = ages.float_data[lastIndex];
				timestamps.float_data[tr] = timestamps.float_data[lastIndex];
				idsTracks.float_data[tr] = idsTracks.float_data[lastIndex];
			}
			//else
			//{
			  //tr = tr + 1;
			//}
			numOfTracks = numOfTracks - 1;
		}
		else
		{
			tr = tr + 1;
		}

	}

	//4) create new tracks
	for (int dp = 0; dp < numOfData; dp = dp + 1)
	{
		//print_line("Datapoint ", temp1, " ", temp2, " matched " , temp3);
		if (isMatchedData.float_data[dp] < 0.5)//0
		{
			//add new track
			if (numOfTracks < maxTracks)
			{
				//print_line("Adding track r=", rData[dp], ", angle=", aData[dp], ", velocity=", vData[dp]);
				set_matrix(states, numOfTracks, 0, rData.float_data[dp]);
				set_matrix(states, numOfTracks, 1, vData.float_data[dp]);
				set_matrix(states, numOfTracks, 2, aData.float_data[dp]);
				set_matrix(states, numOfTracks, 3, 0);
				set_matrix(states, numOfTracks, 4, eData.float_data[dp]);
				set_matrix(states, numOfTracks, 5, 0);
				//x11[numOfTracks] = rData[dp];
				//x21[numOfTracks] = vData[dp];
				//x31[numOfTracks] = aData[dp];
				//x41[numOfTracks] = 0;

				for (int ppos = 0; ppos < 6; ppos = ppos + 1)
				{
					set_matrix_row(covariances, numOfTracks * 6 + ppos, get_matrix_row(Pinit, ppos));
				}

				//p11[numOfTracks] = get_matrix(Pinit, 0,0);
				//p12[numOfTracks] = get_matrix(Pinit, 0,1);
				//p13[numOfTracks] = get_matrix(Pinit, 0,2);
				//p14[numOfTracks] = get_matrix(Pinit, 0,3);

				//p21[numOfTracks] = get_matrix(Pinit, 1,0);
				//p22[numOfTracks] = get_matrix(Pinit, 1,1);
				//p23[numOfTracks] = get_matrix(Pinit, 1,2);
				//p24[numOfTracks] = get_matrix(Pinit, 1,3);

				//p31[numOfTracks] = get_matrix(Pinit, 2,0);
				//p32[numOfTracks] = get_matrix(Pinit, 2,1);
				//p33[numOfTracks] = get_matrix(Pinit, 2,2);
				//p34[numOfTracks] = get_matrix(Pinit, 2,3);

				//p41[numOfTracks] = get_matrix(Pinit, 3,0);
				//p42[numOfTracks] = get_matrix(Pinit, 3,1);
				//p43[numOfTracks] = get_matrix(Pinit, 3,2);
				//p44[numOfTracks] = get_matrix(Pinit, 3,3);

				ages.float_data[numOfTracks] = dt;
				idsTracks.float_data[numOfTracks] = lastTrackId;
				lastTrackId = lastTrackId + 1;
				thresholds.float_data[numOfTracks] = trueTrackThreshold + 0.7*(removeTrackThreshold - trueTrackThreshold);

				numOfTracks = numOfTracks + 1;
			}
		}

	}

	//match tracks
	tr = 0;
	while (tr < numOfTracks)
	{
		tr2 = tr + 1;
		range = get_matrix(states, tr, 0);
		az = get_matrix(states, tr, 2);
		el = get_matrix(states, tr, 4);

		xtrack = range * sin(az * pi() / 180) * cos(el * pi() / 180);
		ytrack = range * cos(az * pi() / 180) * sin(el * pi() / 180);
		ztrack = range * cos(az * pi() / 180) * cos(el * pi() / 180);

		while (tr2 < numOfTracks)
		{
			range2 = get_matrix(states, tr2, 0);
			az2 = get_matrix(states, tr2, 2);
			el2 = get_matrix(states, tr2, 4);

			xtrack2 = range2 * sin(az2 * pi() / 180) * cos(el2 * pi() / 180);
			ytrack2 = range2 * cos(az2 * pi() / 180) * sin(el2 * pi() / 180);
			ztrack2 = range2 * cos(az2 * pi() / 180) * cos(el2 * pi() / 180);

			dpos = sqrt((xtrack2 - xtrack)*(xtrack2 - xtrack) + (ytrack2 - ytrack)*(ytrack2 - ytrack) + (ztrack2 - ztrack)*(ztrack2 - ztrack));

			if (abs(get_matrix(states, tr2, 1)) < 0.01)
			{
				additional = 0.1;
				//print_line("just setting additional");
			}
			else
			{
				//print_line("calculating additional properly");
				additional = 0.1*get_matrix(states, tr2, 1) / abs(get_matrix(states, tr2, 1));
			}

			//print_line("additional ", additional);

			velRatio = get_matrix(states, tr, 1) / (get_matrix(states, tr2, 1) + additional);

			if ((dpos < matchingPositionThreshold) and (velRatio > matchingVelocityThreshold) and (velRatio < 1 / matchingVelocityThreshold))
			{
				//print_line("Merging tracks: ", x11[tr], ", ", x21[tr], ", ", x31[tr], " with ", x11[tr2], ", ", x21[tr2], ", ", x31[tr2]);
				//print_line("x1=", xtrack, ", y1=", ytrack, ", x2=", xtrack2, ", y2=", ytrack2, ", dpos=", dpos);

				set_matrix_row(states, tr, (get_matrix_row(states, tr) + get_matrix_row(states, tr2)) / 2);

				//x11[tr] = (x11[tr] + x11[tr2])/2;
				//x21[tr] = (x21[tr] + x21[tr2])/2;
				//x31[tr] = (x31[tr] + x31[tr2])/2;
				//x41[tr] = (x41[tr] + x41[tr2])/2;

				if (thresholds.float_data[tr] > thresholds.float_data[tr2])
				{
					thresholds.float_data[tr] = thresholds.float_data[tr2];
				}
				if (ages.float_data[tr] < ages.float_data[tr2])
				{
					ages.float_data[tr] = ages.float_data[tr2];
				}

				if (idsTracks.float_data[tr2] < idsTracks.float_data[tr])
				{
					idsTracks.float_data[tr] = idsTracks.float_data[tr2];
				}

				if (tr2 + 1 < numOfTracks)//not last
				{
					lastIndex = numOfTracks - 1;
					set_matrix_row(states, tr2, get_matrix_row(states, lastIndex));

					for (int ppos = 0; ppos < 6; ppos = ppos + 1)
					{
						set_matrix_row(covariances, tr2 * 6 + ppos, get_matrix_row(covariances, lastIndex * 6 + ppos));
					}
					//x11[tr2] = x11[lastIndex];
					//x21[tr2] = x21[lastIndex];
					//x31[tr2] = x31[lastIndex];
					//x41[tr2] = x41[lastIndex];

					//p11[tr2] = p11[lastIndex];
					//p21[tr2] = p21[lastIndex];
					//p31[tr2] = p31[lastIndex];
					//p41[tr2] = p41[lastIndex];

					//p12[tr2] = p12[lastIndex];
					//p22[tr2] = p22[lastIndex];
					//p32[tr2] = p32[lastIndex];
					//p42[tr2] = p42[lastIndex];

					//p13[tr2] = p13[lastIndex];
					//p23[tr2] = p23[lastIndex];
					//p33[tr2] = p33[lastIndex];
					//p43[tr2] = p43[lastIndex];

					//p14[tr2] = p14[lastIndex];
					//p24[tr2] = p24[lastIndex];
					//p34[tr2] = p34[lastIndex];
					//p44[tr2] = p44[lastIndex];

					timestamps.float_data[tr2] = timestamps.float_data[lastIndex];
					ages.float_data[tr2] = ages.float_data[lastIndex];
					thresholds.float_data[tr2] = thresholds.float_data[lastIndex];
					idsTracks.float_data[tr2] = idsTracks.float_data[lastIndex];
				}

				numOfTracks = numOfTracks - 1;
			}
			else
			{
				tr2 = tr2 + 1;
			}
		}

		tr = tr + 1;
	}

	//newLastTrackId = lastTrackId;
	return lastTrackId;
}
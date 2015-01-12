// move this to somewhere
//static inline Vector2d ProjectPtToImg(Matrix3f *A, Matrix4f *H, Vector3f *pt)
//{
//	Vector3f camPt;
//	Vector3f imgPt;
//	Vector2d retPt;
//
//	camPt.x = H->m00 * pt->x + H->m01 * pt->y + H->m02 * pt->z + H->m03;
//	camPt.y = H->m10 * pt->x + H->m11 * pt->y + H->m12 * pt->z + H->m13;
//	camPt.z = H->m20 * pt->x + H->m21 * pt->y + H->m22 * pt->z + H->m23;
//
//	imgPt.x = A->m00 * camPt.x + A->m01 * camPt.y + A->m02 * camPt.z;
//	imgPt.y = A->m10 * camPt.x + A->m11 * camPt.y + A->m12 * camPt.z;
//	imgPt.z = A->m20 * camPt.x + A->m21 * camPt.y + A->m22 * camPt.z;
//
//	retPt.x = (int)(imgPt.x / imgPt.z);
//	retPt.y = (int)(imgPt.y / imgPt.z);
//
//	return retPt;
//}


static inline void UnimindGetMRPfromDegree(float *outR, float *inR)
{
	float rx = inR[0];
	float ry = inR[1];
	float rz = inR[2];

	double rotationX = rx * DEGTORAD;
	double rotationY = ry * DEGTORAD;
	double rotationZ = rz * DEGTORAD;

	double c1 = cos(rotationY / 2);
	double c2 = cos(rotationZ / 2);
	double c3 = cos(rotationX / 2);

	double s1 = sin(rotationY / 2);
	double s2 = sin(rotationZ / 2);
	double s3 = sin(rotationX / 2);

	double c1c2 = c1 * c2;
	double s1s2 = s1 * s2;

	double rotation[4];

	rotation[0] = c1c2*s3 + s1s2*c3;
	rotation[1] = s1*c2*c3 + c1*s2*s3;
	rotation[2] = c1*s2*c3 - s1*c2*s3;
	rotation[3] = c1c2*c3 - s1s2*s3;

	double norm = 1 / sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2] + rotation[3] * rotation[3]);
	for (int i = 0; i<4; i++) rotation[i] *= norm;

	double b0 = rotation[3];
	double b1 = rotation[0];
	double b2 = rotation[1];
	double b3 = rotation[2];

	outR[0] = (float)(b1 / (1 + b0));
	outR[1] = (float)(b2 / (1 + b0));
	outR[2] = (float)(b3 / (1 + b0));
}



static inline void WritePGMimage(char* fileName, unsigned short *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");
	fprintf(fid, "P5\n");
	fprintf(fid, "%d %d\n", w, h);
	fprintf(fid, "65535\n");
	fclose(fid);

	fid = fopen(fileName, "ab+");
	fwrite(imgData, sizeof(unsigned short), w*h, fid);
	fclose(fid);
}

static inline void WritePPMimage(char* fileName, char *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");
	fprintf(fid, "P6\n");
	fprintf(fid, "%d %d\n", w, h);
	fprintf(fid, "255\n");
	fclose(fid);

	fid = fopen(fileName, "ab+");
	fwrite(imgData, sizeof(char), w*h*3, fid);
	fclose(fid);
}

static inline void WriteMatlabTXTImg(char* fileName, float *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int idx = i*w + j;
			fprintf(fid, "%f\t", imgData[idx]);
		}
		fprintf(fid, "\n");
	}
	fclose(fid);
}

static inline void WriteMatlabTXTImg(char* fileName, unsigned char *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int idx = i*w + j;
			fprintf(fid, "%d\t", (int)imgData[idx]);
		}
		fprintf(fid, "\n");
	}
	fclose(fid);
}


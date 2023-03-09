#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>

#define PI 3.14159

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void matrixMultiply(double* R, double* a, double* b)
{
    R[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    R[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    R[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
    R[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    R[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    R[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
    R[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    R[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    R[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
    double X[9] = {1, 0, 0, 0, cos(angles[0]*PI/180.0), -sin(angles[0] * PI / 180.0), 0, sin(angles[0] * PI / 180.0), cos(angles[0] * PI / 180.0)};
    double Y[9] = { cos(angles[1] * PI / 180.0), 0, sin(angles[1] * PI / 180.0), 0, 1, 0, -sin(angles[1] * PI / 180.0), 0, cos(angles[1] * PI / 180.0)};
    double Z[9] = { cos(angles[2] * PI / 180.0), -sin(angles[2] * PI / 180.0), 0, sin(angles[2] * PI / 180.0), cos(angles[2] * PI / 180.0), 0, 0, 0, 1 };
    double YX[9];
    matrixMultiply(YX, Y, X);
    matrixMultiply(R, Z, YX);
}

vector EulerDistance(const vector& a, const vector& b, double u)
{
    return a + (b - a) * u;
}

void Interpolator::EulerFormControlPosition(Motion* pInputMotion, int startFrame, int N, vector& an, vector& bn)
{
    int n = startFrame;
    int n_plus_one = startFrame + N + 1;
    int n_plus_two = n_plus_one + N + 1;
    int n_minus_one = startFrame - N - 1;
    int n_minus_two = n_minus_one - N - 1;

    Posture* e = pInputMotion->GetPosture(n);
    Posture* e_plus_one, * e_plus_two, * e_minus_one, * e_minus_two;
    if (n_minus_one < 0)
    {
        e_plus_one = pInputMotion->GetPosture(n_plus_one);
        e_plus_two = pInputMotion->GetPosture(n_plus_two);
    }
    else if(n_plus_one > pInputMotion->GetNumFrames())
    {
        e_minus_one = pInputMotion->GetPosture(n_minus_one);
        e_minus_two = pInputMotion->GetPosture(n_minus_two);
    }
    else
    {
        e_minus_one = pInputMotion->GetPosture(n_minus_one);
        e_plus_one = pInputMotion->GetPosture(n_plus_one);
    }

    if (n_minus_one < 0)
    {
        // which means we never have to compute bn
        an = EulerDistance(e->root_pos, EulerDistance(e_plus_two->root_pos, e_plus_one->root_pos, 2.0), 1.0/3);
    }
    else if (n_plus_one > pInputMotion->GetNumFrames())
    {
        // which means we never have to compute an
        bn = EulerDistance(e->root_pos, EulerDistance(e_minus_two->root_pos, e_minus_one->root_pos, 2.0), 1.0 / 3);
    }
    else
    {
        // an and bn should be computed 
        vector an_bar = EulerDistance(EulerDistance(e_minus_one->root_pos, e->root_pos, 2.0), e_plus_one->root_pos, 0.5);
        an = EulerDistance(e->root_pos, an_bar, 1.0 / 3.0);
        bn = EulerDistance(e->root_pos, an_bar, -1.0 / 3.0);
    }
}

void Interpolator::EulerFormControlPoint(Motion* pInputMotion, int startFrame, int N, vector* an, vector* bn)
{
    int n = startFrame;
    int n_plus_one = startFrame + N + 1;
    int n_plus_two = n_plus_one + N + 1;
    int n_minus_one = startFrame - N - 1;
    int n_minus_two = n_minus_one - N - 1;

    Posture* e = pInputMotion->GetPosture(n);
    Posture* e_plus_one, * e_plus_two, * e_minus_one, * e_minus_two;
    if (n_minus_one < 0)
    {
        e_plus_one = pInputMotion->GetPosture(n_plus_one);
        e_plus_two = pInputMotion->GetPosture(n_plus_two);
    }
    else if(n_plus_one > pInputMotion->GetNumFrames())
    {
        e_minus_one = pInputMotion->GetPosture(n_minus_one);
        e_minus_two = pInputMotion->GetPosture(n_minus_two);
    }
    else
    {
        e_minus_one = pInputMotion->GetPosture(n_minus_one);
        e_plus_one = pInputMotion->GetPosture(n_plus_one);
    }
    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
    {
        if (n_minus_one < 0)
        {
            // which means we never have to compute bn
            an[bone] = EulerDistance(e->bone_rotation[bone], EulerDistance(e_plus_two->bone_rotation[bone], e_plus_one->bone_rotation[bone], 2.0), 1.0/3);
        }
        else if (n_plus_one > pInputMotion->GetNumFrames())
        {
            // which means we never have to compute an
            bn[bone] = EulerDistance(e->bone_rotation[bone], EulerDistance(e_minus_two->bone_rotation[bone], e_minus_one->bone_rotation[bone], 2.0), 1.0 / 3);
        }
        else
        {
            // an and bn should be computed 
            vector an_bar = EulerDistance(EulerDistance(e_minus_one->bone_rotation[bone], e->bone_rotation[bone], 2.0), e_plus_one->bone_rotation[bone], 0.5);
            an[bone] = EulerDistance(e->bone_rotation[bone], an_bar, 1.0 / 3.0);
            bn[bone] = EulerDistance(e->bone_rotation[bone], an_bar, -1.0 / 3.0);
        }
    }
}

void Interpolator::QuaternionFormControlPoint(Motion* pInputMotion, int startFrame, int N, std::vector<Quaternion<double>>& an, std::vector<Quaternion<double>>& bn)
{
    int n = startFrame;
    int n_plus_one = startFrame + N + 1;
    int n_plus_two = n_plus_one + N + 1;
    int n_minus_one = startFrame - N - 1;
    int n_minus_two = n_minus_one - N - 1;

    Posture* e = pInputMotion->GetPosture(n);
    Posture* e_plus_one, * e_plus_two, * e_minus_one, * e_minus_two;
    if (n_minus_one < 0)
    {
        e_plus_one = pInputMotion->GetPosture(n_plus_one);
        e_plus_two = pInputMotion->GetPosture(n_plus_two);
    }
    else if (n_plus_one > pInputMotion->GetNumFrames())
    {
        e_minus_one = pInputMotion->GetPosture(n_minus_one);
        e_minus_two = pInputMotion->GetPosture(n_minus_two);
    }
    else
    {
        e_minus_one = pInputMotion->GetPosture(n_minus_one);
        e_plus_one = pInputMotion->GetPosture(n_plus_one);
    }
    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
    {
        if (n_minus_one < 0)
        {
            // which means we never have to compute bn
            Quaternion<double> q_plus_two, q_plus_one, q;
            Euler2Quaternion(e->bone_rotation[bone].p, q);
            Euler2Quaternion(e_plus_one->bone_rotation[bone].p, q_plus_one);
            Euler2Quaternion(e_plus_two->bone_rotation[bone].p, q_plus_two);
            an[bone] = Slerp(1.0 / 3, q, Double(q_plus_two, q_plus_one));
        }
        else if (n_plus_one > pInputMotion->GetNumFrames())
        {
            // which means we never have to compute an
            Quaternion<double> q_minus_two, q_minus_one, q;
            Euler2Quaternion(e->bone_rotation[bone].p, q);
            Euler2Quaternion(e_minus_one->bone_rotation[bone].p, q_minus_one);
            Euler2Quaternion(e_minus_two->bone_rotation[bone].p, q_minus_two);
            bn[bone] = Slerp(1.0 / 3, q, Double(q_minus_two, q_minus_one));
        }
        else
        {
            // an and bn should be computed 
            Quaternion<double> q_minus_one, q_plus_one, q;
            Euler2Quaternion(e->bone_rotation[bone].p, q);
            Euler2Quaternion(e_plus_one->bone_rotation[bone].p, q_plus_one);
            Euler2Quaternion(e_minus_one->bone_rotation[bone].p, q_minus_one);
            Quaternion<double> an_bar = Slerp(0.5, Double(q_minus_one, q), q_plus_one);
            an[bone] = Slerp(1.0/3, q, an_bar);
            bn[bone] = Slerp(-1.0/3, q, an_bar);
        }
    }
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    /*
        First to form two control points
        second use DeCasteljauEuler to get the result
    */
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    bool initialize = false;
    vector* an = new vector[MAX_BONES_IN_ASF_FILE];
    vector* bn = new vector[MAX_BONES_IN_ASF_FILE];
    vector* an_plus_one = new vector[MAX_BONES_IN_ASF_FILE];
    vector* bn_plus_one = new vector[MAX_BONES_IN_ASF_FILE];
    vector apos, bpos;
    vector apos_plus_one, bpos_plus_one;

    while (startKeyframe + N + 1 < inputLength)
    {
        std::cout << "startKeyFrame is" << startKeyframe << std::endl;
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // we have to get q_n-1, q_n, q_n+1 to compute a_n, b_n
        // however, we need q_n, q_n+1, a_n, b_n+1 to interpolate for q_n
        // here we design extra functions for compute a_n, b_n to ensure the structure of code is clear
        // moreover, to efficiently storage and compute the data, we will also store a_n+1, b_n+1 and do the swap each iteration

        if (!initialize)
        {
            initialize = true;
            EulerFormControlPoint(pInputMotion, startKeyframe, N, an, bn);
            EulerFormControlPosition(pInputMotion, startKeyframe, N, apos, bpos);
        }

        // Now for each iteration, only computes n for endKeyFrame
        EulerFormControlPoint(pInputMotion, endKeyframe, N, an_plus_one, bn_plus_one);
        EulerFormControlPosition(pInputMotion, endKeyframe, N, apos_plus_one, bpos_plus_one);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position in Bezier
            interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, apos, bpos_plus_one, endPosture->root_pos);

            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // maybe we can do it here, for each bone, we compute a_n, b_n+1
                // for now, we use v_before, v_current, v_after to represent three consecutive vectors of pos we sample;
                vector p0 = startPosture->bone_rotation[bone];
                vector p3 = endPosture->bone_rotation[bone];
                vector p1 = an[bone];
                vector p2 = bn_plus_one[bone];
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p0, p1, p2, p3);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
        //swapping
        for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {
            an[bone] = an_plus_one[bone];
            bn[bone] = bn_plus_one[bone];
        }
        apos = apos_plus_one;
        bpos = bpos_plus_one;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    delete[]an;
    delete[]bn;
    delete[]an_plus_one;
    delete[]bn_plus_one;
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // interpolate root position Euler
                // transform euler to rotation to quaternion
                Quaternion<double> startQ, endQ, q;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQ);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQ);
                q = Slerp(t, startQ, endQ);
                double angles[3];
                Quaternion2Euler(q, angles);
                interpolatedPosture.bone_rotation[bone].setValue(angles);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    std::vector<Quaternion<double>> an;
    std::vector<Quaternion<double>> bn;
    std::vector<Quaternion<double>> an_plus_one;
    std::vector<Quaternion<double>> bn_plus_one;
    an.resize(MAX_BONES_IN_ASF_FILE);
    bn.resize(MAX_BONES_IN_ASF_FILE);
    an_plus_one.resize(MAX_BONES_IN_ASF_FILE);
    bn_plus_one.resize(MAX_BONES_IN_ASF_FILE);
    vector apos, bpos;
    vector apos_plus_one, bpos_plus_one;
    bool initialize = false;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // we have to get q_n-1, q_n, q_n+1 to compute a_n, b_n
        // however, we need q_n, q_n+1, a_n, b_n+1 to interpolate for q_n
        // here we design extra functions for compute a_n, b_n to ensure the structure of code is clear
        // moreover, to efficiently storage and compute the data, we will also store a_n+1, b_n+1 and do the swap each iteration

        if (!initialize)
        {
            initialize = true;
            QuaternionFormControlPoint(pInputMotion, startKeyframe, N, an, bn);
            EulerFormControlPosition(pInputMotion, startKeyframe, N, apos, bpos);
        }

        // Now for each iteration, only computes n for endKeyFrame
        QuaternionFormControlPoint(pInputMotion, endKeyframe, N, an_plus_one, bn_plus_one);
        EulerFormControlPosition(pInputMotion, endKeyframe, N, apos_plus_one, bpos_plus_one);


        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, apos, bpos_plus_one, endPosture->root_pos);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // interpolate root position Euler
                // transform euler to rotation to quaternion
                Quaternion<double> startQ, endQ, q;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, startQ);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, endQ);

                q = DeCasteljauQuaternion(t, startQ, an[bone], bn_plus_one[bone], endQ);

                //back to eulers
                double angles[3];
                Quaternion2Euler(q, angles);
                interpolatedPosture.bone_rotation[bone].setValue(angles);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
        //swapping
        std::swap(an, an_plus_one);
        std::swap(bn, bn_plus_one);
        apos = apos_plus_one;
        bpos = bpos_plus_one;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
    // euler to rotation first
    double R[9];
    Euler2Rotation(angles, R);
    q = Quaternion<double>::Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
    // quaternion to rotation first
    double R[9];
    q.Quaternion2Matrix(R);
    Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  double dot = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();
  dot = std::max(std::min(dot, 1.0), -1.0);
  if (dot < 0)
  {
      //flip quaternion
      qStart = -1 * qStart;
  }
  double theta = std::acos(std::abs(dot));
  if (theta == 0.0 || theta == 180.0)
      return qStart;
  return (std::sin((1-t)*theta)/sin(theta))*qStart + (std::sin(t * theta) / sin(theta)) * qEnd_;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  return Slerp(2.0, p, q);
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  // use helper function
  vector q0 = EulerDistance(p0, p1, t);
  vector q1 = EulerDistance(p1, p2, t);
  vector q2 = EulerDistance(p2, p3, t);
  vector r0 = EulerDistance(q0, q1, t);
  vector r1 = EulerDistance(q1, q2, t);
  result = EulerDistance(r0, r1, t);
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> q0 = Slerp(t, p0, p1);
  Quaternion<double> q1 = Slerp(t, p1, p2);
  Quaternion<double> q2 = Slerp(t, p2, p3);
  Quaternion<double> r0 = Slerp(t, q0, q1);
  Quaternion<double> r1 = Slerp(t, q1, q2);
  return Slerp(t, r0, r1);
}


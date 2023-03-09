#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>

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
    
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
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

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
                interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

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
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
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
  return result;
}


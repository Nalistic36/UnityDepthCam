using UnityEngine;

using System;
using System.Collections;
using System.Collections.Generic;

public class SimpleKinectManager : MonoBehaviour
{
	public enum Smoothing : int { None, Default, Medium, Aggressive }

	// How high off the ground is the sensor (in meters).
	public float SensorHeight = 1.0f;

	// Kinect elevation angle (in degrees)
	public int SensorAngle = 0;

	// Minimum user distance in order to process skeleton data
	public float MinUserDistance = 0.5f;

	// Maximum user distance, if any. 0 means no max-distance limitation
	public float MaxUserDistance = 0f;

	// Public Bool to determine whether to detect only the closest user or not
	public bool DetectClosestUser = true;

	// Public Bool to determine whether to use only the tracked joints (and ignore the inferred ones)
	public bool IgnoreInferredJoints = true;

	// Selection of smoothing parameters
	public Smoothing smoothing = Smoothing.Default;

	// Public Bool to determine the use of additional filters
	public bool UseBoneOrientationsFilter = false;
	public bool UseBoneOrientationsConstraint = true;
	public bool UseSelfIntersectionConstraint = false;

	// Bool to keep track of whether Kinect has been initialized
	private bool KinectInitialized = false;

	// Bools to keep track of who is currently calibrated.
	private bool Player1Calibrated = false;

	private bool AllPlayersCalibrated = false;

	// Values to track which ID (assigned by the Kinect) is player 1 and player 2.
	private uint Player1ID;

	private int Player1Index;

	private Color32[] usersMapColors;
	private ushort[] usersPrevState;
	private Rect usersMapRect;
	private int usersMapSize;

	private Rect usersClrRect;

	private float[] usersHistogramMap;

	// List of all users
	private List<uint> allUsers;

	// Image stream handles for the kinect
	private IntPtr colorStreamHandle;
	private IntPtr depthStreamHandle;

	// Skeleton related structures
	private KinectWrapper.NuiSkeletonFrame skeletonFrame;
	private KinectWrapper.NuiTransformSmoothParameters smoothParameters;
	private int player1Index;

	// Skeleton tracking states, positions and joints' orientations
	private Vector3 player1Pos;
	private Matrix4x4 player1Ori;
	private bool[] player1JointsTracked;
	private bool[] player1PrevTracked;
	private Vector3[] player1JointsPos;
	private Matrix4x4[] player1JointsOri;
	private KinectWrapper.NuiSkeletonBoneOrientation[] jointOrientations;

	// general gesture tracking time start
	private float[] gestureTrackingAtTime;

	private Matrix4x4 kinectToWorld, flipMatrix;

	// Timer for controlling Filter Lerp blends.
	private float lastNuiTime;

	// Filters
	private TrackingStateFilter[] trackingStateFilter;
	private BoneOrientationsFilter[] boneOrientationFilter;
	private ClippedLegsFilter[] clippedLegsFilter;
	private BoneOrientationsConstraint boneConstraintsFilter;
	private SelfIntersectionConstraint selfIntersectionConstraint;

	// checks if Kinect is initialized and ready to use. If not, there was an error during Kinect-sensor initialization
	public bool IsInitialized()
	{
		return KinectInitialized;
	}

	// this function is used internally by AvatarController
	public static bool IsCalibrationNeeded()
	{
		return false;
	}

	// returns the depth map position for a 3d joint position
	public Vector2 GetDepthMapPosForJointPos(Vector3 posJoint)
	{
		Vector3 vDepthPos = KinectWrapper.MapSkeletonPointToDepthPoint(posJoint);
		Vector2 vMapPos = new Vector2(vDepthPos.x, vDepthPos.y);

		return vMapPos;
	}

	// returns true if at least one user is currently detected by the sensor
	public bool IsUserDetected()
	{
		return KinectInitialized && (allUsers.Count > 0);
	}

	// returns the UserID of Player1, or 0 if no Player1 is detected
	public uint GetPlayer1ID()
	{
		return Player1ID;
	}

	// returns the index of Player1, or 0 if no is detected
	public int GetPlayer1Index()
	{
		return Player1Index;
	}

	// returns true if the User is calibrated and ready to use
	public bool IsPlayerCalibrated(uint UserId)
	{
		if (UserId == Player1ID)
			return Player1Calibrated;

		return false;
	}

	// returns the raw unmodified joint position, as returned by the Kinect sensor
	public Vector3 GetRawSkeletonJointPos(uint UserId, int joint)
	{
		if (UserId == Player1ID)
			return joint >= 0 && joint < player1JointsPos.Length ? (Vector3)skeletonFrame.SkeletonData[player1Index].SkeletonPositions[joint] : Vector3.zero;
	
		return Vector3.zero;
	}

	// returns the User position, relative to the Kinect-sensor, in meters
	public Vector3 GetUserPosition(uint UserId)
	{
		if (UserId == Player1ID)
			return player1Pos;
		
		return Vector3.zero;
	}

	// returns the User rotation, relative to the Kinect-sensor
	public Quaternion GetUserOrientation(uint UserId, bool flip)
	{
		if (UserId == Player1ID && player1JointsTracked[(int)KinectWrapper.NuiSkeletonPositionIndex.HipCenter])
			return ConvertMatrixToQuat(player1Ori, (int)KinectWrapper.NuiSkeletonPositionIndex.HipCenter, flip);

		return Quaternion.identity;
	}

	// returns true if the given joint of the specified user is being tracked
	public bool IsJointTracked(int joint)
	{
	    return joint >= 0 && joint < player1JointsTracked.Length ? player1JointsTracked[joint] : false;
	}

	// returns the joint position of the specified user, relative to the Kinect-sensor, in meters
	public Vector3 GetJointPosition(int joint)
	{
		return joint >= 0 && joint < player1JointsPos.Length ? player1JointsPos[joint] : Vector3.zero;
	}

	// returns the local joint position of the specified user, relative to the parent joint, in meters
	public Vector3 GetJointLocalPosition(int joint)
	{
		int parent = KinectWrapper.GetSkeletonJointParent(joint);

		return joint >= 0 && joint < player1JointsPos.Length ?
			(player1JointsPos[joint] - player1JointsPos[parent]) : Vector3.zero;
	}

	// returns the joint rotation of the specified user, relative to the Kinect-sensor
	public Quaternion GetJointOrientation(int joint, bool flip = true)
	{
		if (joint >= 0 && joint < player1JointsOri.Length && player1JointsTracked[joint])
			return ConvertMatrixToQuat(player1JointsOri[joint], joint, flip);

		return Quaternion.identity;
	}

	// returns the joint rotation of the specified user, relative to the parent joint
	public Quaternion GetJointLocalOrientation(uint UserId, int joint, bool flip)
	{
		int parent = KinectWrapper.GetSkeletonJointParent(joint);

		if (UserId == Player1ID)
		{
			if (joint >= 0 && joint < player1JointsOri.Length && player1JointsTracked[joint])
			{
				Matrix4x4 localMat = (player1JointsOri[parent].inverse * player1JointsOri[joint]);
				return Quaternion.LookRotation(localMat.GetColumn(2), localMat.GetColumn(1));
			}
		}

		return Quaternion.identity;
	}

	// returns the direction between baseJoint and nextJoint, for the specified user
	public Vector3 GetDirectionBetweenJoints(int baseJoint, int nextJoint, bool flipX, bool flipZ)
	{
		Vector3 jointDir = Vector3.zero;

		if (baseJoint >= 0 && baseJoint < player1JointsPos.Length && player1JointsTracked[baseJoint] &&
			nextJoint >= 0 && nextJoint < player1JointsPos.Length && player1JointsTracked[nextJoint])
		{
			jointDir = player1JointsPos[nextJoint] - player1JointsPos[baseJoint];
		}
	
		if (jointDir != Vector3.zero)
		{
			if (flipX)
				jointDir.x = -jointDir.x;

			if (flipZ)
				jointDir.z = -jointDir.z;
		}

		return jointDir;
	}

	public void ClearKinectUsers()
	{
		if (!KinectInitialized)
			return;

		// remove current users
		for (int i = allUsers.Count - 1; i >= 0; i--)
		{
			uint userId = allUsers[i];
			RemoveUser(userId);
		}

		ResetFilters();
	}

	// clears Kinect buffers and resets the filters
	public void ResetFilters()
	{
		if (!KinectInitialized)
			return;

		// clear kinect vars
		player1Pos = Vector3.zero;
		player1Ori = Matrix4x4.identity;

		int skeletonJointsCount = (int)KinectWrapper.NuiSkeletonPositionIndex.Count;
		for (int i = 0; i < skeletonJointsCount; i++)
		{
			player1JointsTracked[i] = false;
			player1PrevTracked[i] = false;
			player1JointsPos[i] = Vector3.zero;
			player1JointsOri[i] = Matrix4x4.identity;
		}

		if (trackingStateFilter != null)
		{
			for (int i = 0; i < trackingStateFilter.Length; i++)
				if (trackingStateFilter[i] != null)
					trackingStateFilter[i].Reset();
		}

		if (boneOrientationFilter != null)
		{
			for (int i = 0; i < boneOrientationFilter.Length; i++)
				if (boneOrientationFilter[i] != null)
					boneOrientationFilter[i].Reset();
		}

		if (clippedLegsFilter != null)
		{
			for (int i = 0; i < clippedLegsFilter.Length; i++)
				if (clippedLegsFilter[i] != null)
					clippedLegsFilter[i].Reset();
		}
	}

	void Awake()
	{
        WrapperTools.EnsureKinectWrapperPresence();
        int hr = 0;

		try
		{
			hr = KinectWrapper.NuiInitialize(KinectWrapper.NuiInitializeFlags.UsesSkeleton |
				KinectWrapper.NuiInitializeFlags.UsesDepthAndPlayerIndex);
			if (hr != 0)
			{
				throw new Exception("NuiInitialize Failed");
			}

			hr = KinectWrapper.NuiSkeletonTrackingEnable(IntPtr.Zero, 8);  // 0, 12,8
			if (hr != 0)
			{
				throw new Exception("Cannot initialize Skeleton Data");
			}

			depthStreamHandle = IntPtr.Zero;

			// set kinect elevation angle
			KinectWrapper.NuiCameraElevationSetAngle(SensorAngle);

			// init skeleton structures
			skeletonFrame = new KinectWrapper.NuiSkeletonFrame()
			{
				SkeletonData = new KinectWrapper.NuiSkeletonData[KinectWrapper.Constants.NuiSkeletonCount]
			};

			// values used to pass to smoothing function
			smoothParameters = new KinectWrapper.NuiTransformSmoothParameters();

			switch (smoothing)
			{
				case Smoothing.Default:
					smoothParameters.fSmoothing = 0.5f;
					smoothParameters.fCorrection = 0.5f;
					smoothParameters.fPrediction = 0.5f;
					smoothParameters.fJitterRadius = 0.05f;
					smoothParameters.fMaxDeviationRadius = 0.04f;
					break;
				case Smoothing.Medium:
					smoothParameters.fSmoothing = 0.5f;
					smoothParameters.fCorrection = 0.1f;
					smoothParameters.fPrediction = 0.5f;
					smoothParameters.fJitterRadius = 0.1f;
					smoothParameters.fMaxDeviationRadius = 0.1f;
					break;
				case Smoothing.Aggressive:
					smoothParameters.fSmoothing = 0.7f;
					smoothParameters.fCorrection = 0.3f;
					smoothParameters.fPrediction = 1.0f;
					smoothParameters.fJitterRadius = 1.0f;
					smoothParameters.fMaxDeviationRadius = 1.0f;
					break;
			}

			// init the tracking state filter
			trackingStateFilter = new TrackingStateFilter[KinectWrapper.Constants.NuiSkeletonMaxTracked];
			for (int i = 0; i < trackingStateFilter.Length; i++)
			{
				trackingStateFilter[i] = new TrackingStateFilter();
				trackingStateFilter[i].Init();
			}

			// init the bone orientation filter
			boneOrientationFilter = new BoneOrientationsFilter[KinectWrapper.Constants.NuiSkeletonMaxTracked];
			for (int i = 0; i < boneOrientationFilter.Length; i++)
			{
				boneOrientationFilter[i] = new BoneOrientationsFilter();
				boneOrientationFilter[i].Init();
			}

			// init the clipped legs filter
			clippedLegsFilter = new ClippedLegsFilter[KinectWrapper.Constants.NuiSkeletonMaxTracked];
			for (int i = 0; i < clippedLegsFilter.Length; i++)
			{
				clippedLegsFilter[i] = new ClippedLegsFilter();
			}

			// init the bone orientation constraints
			boneConstraintsFilter = new BoneOrientationsConstraint();
			boneConstraintsFilter.AddDefaultConstraints();
			// init the self intersection constraints
			selfIntersectionConstraint = new SelfIntersectionConstraint();

			// create arrays for joint positions and joint orientations
			int skeletonJointsCount = (int)KinectWrapper.NuiSkeletonPositionIndex.Count;

			player1JointsTracked = new bool[skeletonJointsCount];
			player1PrevTracked = new bool[skeletonJointsCount];

			player1JointsPos = new Vector3[skeletonJointsCount];

			player1JointsOri = new Matrix4x4[skeletonJointsCount];

			gestureTrackingAtTime = new float[KinectWrapper.Constants.NuiSkeletonMaxTracked];

			//create the transform matrix that converts from kinect-space to world-space
			Quaternion quatTiltAngle = new Quaternion();
			quatTiltAngle.eulerAngles = new Vector3(-SensorAngle, 0.0f, 0.0f);

			//float heightAboveHips = SensorHeight - 1.0f;

			// transform matrix - kinect to world
			kinectToWorld.SetTRS(new Vector3(0.0f, SensorHeight, 0.0f), quatTiltAngle, Vector3.one);
			flipMatrix = Matrix4x4.identity;
			flipMatrix[2, 2] = -1;
		}
		catch (DllNotFoundException e)
		{
			string message = "Please check the Kinect SDK installation.";
			Debug.LogError(message);
			Debug.LogError(e.ToString());

			return;
		}
		catch (Exception e)
		{
			string message = e.Message + " - " + KinectWrapper.GetNuiErrorString(hr);
			Debug.LogError(message);
			Debug.LogError(e.ToString());

			return;
		}

		// Initialize user list to contain ALL users.
		allUsers = new List<uint>();

		KinectInitialized = true;
	}

	void Update()
	{
		if (KinectInitialized)
		{
			if (KinectWrapper.PollSkeleton(ref smoothParameters, ref skeletonFrame))
			{
				ProcessSkeleton();
			}	
		}
	}

	// Make sure to kill the Kinect on quitting.
	void OnApplicationQuit()
	{
		if (KinectInitialized)
		{
			// Shutdown OpenNI
			KinectWrapper.NuiShutdown();
		}
	}

	// Assign UserId to player 1 or 2.
	void CalibrateUser(uint UserId, int UserIndex, ref KinectWrapper.NuiSkeletonData skeletonData)
	{
		// If player 1 hasn't been calibrated, assign that UserID to it.
		if (!Player1Calibrated)
		{
			// Check to make sure we don't accidentally assign player 2 to player 1.
			if (!allUsers.Contains(UserId))
			{
				Player1Calibrated = true;
				Player1ID = UserId;
				Player1Index = UserIndex;

				allUsers.Add(UserId);

				// reset skeleton filters
				ResetFilters();
			    AllPlayersCalibrated = allUsers.Count >= 1;
			}
		}
		
		// If all users are calibrated, stop trying to find them.
		if (AllPlayersCalibrated)
		{
			Debug.Log("All players calibrated.");
		}
	}

	// Remove a lost UserId
	void RemoveUser(uint UserId)
	{
		// If we lose player 1...
		if (UserId == Player1ID)
		{
			// Null out the ID and reset all the models associated with that ID.
			Player1ID = 0;
			Player1Index = 0;
			Player1Calibrated = false;
		}	

		// remove from global users list
		allUsers.Remove(UserId);
		AllPlayersCalibrated = allUsers.Count >= 1;
	}

	// Some internal constants
	private const int stateTracked = (int)KinectWrapper.NuiSkeletonPositionTrackingState.Tracked;
	private const int stateNotTracked = (int)KinectWrapper.NuiSkeletonPositionTrackingState.NotTracked;

	private int[] mustBeTrackedJoints = {
        (int)KinectWrapper.NuiSkeletonPositionIndex.Head
		//(int)KinectWrapper.NuiSkeletonPositionIndex.AnkleLeft,
		//(int)KinectWrapper.NuiSkeletonPositionIndex.FootLeft,
		//(int)KinectWrapper.NuiSkeletonPositionIndex.AnkleRight,
		//(int)KinectWrapper.NuiSkeletonPositionIndex.FootRight,
	};

	// Process the skeleton data
	void ProcessSkeleton()
	{
		List<uint> lostUsers = new List<uint>();
		lostUsers.AddRange(allUsers);

		// calculate the time since last update
		float currentNuiTime = Time.realtimeSinceStartup;
		float deltaNuiTime = currentNuiTime - lastNuiTime;

		for (int i = 0; i < KinectWrapper.Constants.NuiSkeletonCount; i++)
		{
			KinectWrapper.NuiSkeletonData skeletonData = skeletonFrame.SkeletonData[i];
			uint userId = skeletonData.dwTrackingID;

			if (skeletonData.eTrackingState == KinectWrapper.NuiSkeletonTrackingState.SkeletonTracked)
			{
				// get the skeleton position
				Vector3 skeletonPos = kinectToWorld.MultiplyPoint3x4(skeletonData.Position);

				if (!AllPlayersCalibrated)
				{
					// check if this is the closest user
					bool bClosestUser = true;

					if (DetectClosestUser)
					{
						for (int j = 0; j < KinectWrapper.Constants.NuiSkeletonCount; j++)
						{
							if (j != i)
							{
								KinectWrapper.NuiSkeletonData skeletonDataOther = skeletonFrame.SkeletonData[j];

								if ((skeletonDataOther.eTrackingState == KinectWrapper.NuiSkeletonTrackingState.SkeletonTracked) &&
									(Mathf.Abs(kinectToWorld.MultiplyPoint3x4(skeletonDataOther.Position).z) < Mathf.Abs(skeletonPos.z)))
								{
									bClosestUser = false;
									break;
								}
							}
						}
					}

					if (bClosestUser)
					{
						CalibrateUser(userId, i + 1, ref skeletonData);
					}
				}

				if (userId == Player1ID && Mathf.Abs(skeletonPos.z) >= MinUserDistance &&
				   (MaxUserDistance <= 0f || Mathf.Abs(skeletonPos.z) <= MaxUserDistance))
				{
					player1Index = i;

					// get player position
					player1Pos = skeletonPos;

					// apply tracking state filter first
					trackingStateFilter[0].UpdateFilter(ref skeletonData);

					if (UseSelfIntersectionConstraint && selfIntersectionConstraint != null)
					{
						selfIntersectionConstraint.Constrain(ref skeletonData);
					}

					// get joints' position and rotation
					for (int j = 0; j < (int)KinectWrapper.NuiSkeletonPositionIndex.Count; j++)
					{
						bool playerTracked = IgnoreInferredJoints ? (int)skeletonData.eSkeletonPositionTrackingState[j] == stateTracked :
							(Array.BinarySearch(mustBeTrackedJoints, j) >= 0 ? (int)skeletonData.eSkeletonPositionTrackingState[j] == stateTracked :
							(int)skeletonData.eSkeletonPositionTrackingState[j] != stateNotTracked);
						player1JointsTracked[j] = player1PrevTracked[j] && playerTracked;
						player1PrevTracked[j] = playerTracked;

						if (player1JointsTracked[j])
						{
							player1JointsPos[j] = kinectToWorld.MultiplyPoint3x4(skeletonData.SkeletonPositions[j]);
							//player1JointsOri[j] = jointOrients[j].absoluteRotation.rotationMatrix * flipMatrix;
						}
					}

					// calculate joint orientations
					KinectWrapper.GetSkeletonJointOrientation(ref player1JointsPos, ref player1JointsTracked, ref player1JointsOri);

					// filter orientation constraints
					if (UseBoneOrientationsConstraint && boneConstraintsFilter != null)
					{
						boneConstraintsFilter.Constrain(ref player1JointsOri, ref player1JointsTracked);
					}

					// filter joint orientations.
					// it should be performed after all joint position modifications.
					if (UseBoneOrientationsFilter && boneOrientationFilter[0] != null)
					{
						boneOrientationFilter[0].UpdateFilter(ref skeletonData, ref player1JointsOri);
					}

					// get player rotation
					player1Ori = player1JointsOri[(int)KinectWrapper.NuiSkeletonPositionIndex.HipCenter];
				}
				
				lostUsers.Remove(userId);
			}
		}

		// update the nui-timer
		lastNuiTime = currentNuiTime;

		// remove the lost users if any
		if (lostUsers.Count > 0)
		{
			foreach (uint userId in lostUsers)
			{
				RemoveUser(userId);
			}

			lostUsers.Clear();
		}
	}

	// convert the matrix to quaternion, taking care of the mirroring
	private Quaternion ConvertMatrixToQuat(Matrix4x4 mOrient, int joint, bool flip)
	{
		Vector4 vZ = mOrient.GetColumn(2);
		Vector4 vY = mOrient.GetColumn(1);

		if (!flip)
		{
			vZ.y = -vZ.y;
			vY.x = -vY.x;
			vY.z = -vY.z;
		}
		else
		{
			vZ.x = -vZ.x;
			vZ.y = -vZ.y;
			vY.z = -vY.z;
		}

		if (vZ.x != 0.0f || vZ.y != 0.0f || vZ.z != 0.0f)
			return Quaternion.LookRotation(vZ, vY);
		else
			return Quaternion.identity;
	}
}
using UnityEngine;

[RequireComponent(typeof(SimpleKinectManager))]
public class HeadTracking : MonoBehaviour
{
    private SimpleKinectManager _manager;
    private int _head;

    void Awake()
    {
        _manager = GetComponent<SimpleKinectManager>();
        _head = (int)KinectWrapper.NuiSkeletonPositionIndex.Head;
    }

    private void Update()
    {
        Debug.Log(UpdateHeadPose());
    }

    private Vector3 UpdateHeadPose()
    {
        return _manager.GetJointPosition(_head);
    }
}

using UnityEngine;

[RequireComponent(typeof(SimpleKinectManager))]
public class HeadTracking : MonoBehaviour
{
    private SimpleKinectManager _manager;
    private Transform _cameraTransform;
    private int _head;

    void Awake()
    {
        _manager = GetComponent<SimpleKinectManager>();
        _head = (int)KinectWrapper.NuiSkeletonPositionIndex.Head;
        _cameraTransform = Camera.main.transform;
    }

    private void Update()
    {
        if(_manager.IsInitialized())
        {
            _cameraTransform.position = UpdateHeadPose();
            //_cameraTransform.rotation = UpdateHeadOrientation();
            Debug.Log(UpdateHeadOrientation());
        }
    }

    private Vector3 UpdateHeadPose()
    {
        return _manager.GetJointPosition(_head);
    }

    private Quaternion UpdateHeadOrientation()
    {
        return _manager.GetJointOrientation(_head);
    }
}

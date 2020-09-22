using UnityEngine;

[RequireComponent(typeof(SimpleKinectManager))]
public class HeadTracking : MonoBehaviour
{
    private SimpleKinectManager _manager;
    private Transform _cameraTransform;
    private int _head;

    [SerializeField] private float positionSpeed = 1.0f;
    [SerializeField] private float rotationSpeed = 1.0f;

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
            _cameraTransform.position = Vector3.MoveTowards(_cameraTransform.position, UpdateHeadPose(), positionSpeed * Time.deltaTime);
            _cameraTransform.rotation = Quaternion.RotateTowards(_cameraTransform.rotation, UpdateHeadOrientation(), rotationSpeed * Time.deltaTime);
        }

        //Debug.Log(_manager.IsJointTracked(_head));
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

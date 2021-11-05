using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Move : MonoBehaviour
{
    public Transform Target;
    [Range(0, 1)]
    public float Speed = .1f;

    Transform trans;
    Rigidbody rig;
    PathFinding findPath;

    private void Awake()
    {
        trans = transform;
        rig = GetComponent<Rigidbody>();
        findPath = GetComponent<PathFinding>();
    }

    private void Start()
    {
        if(Target != null)
            findPath.FindPathTo(Target, Target.GetComponent<Collider>());
    }

    private void FixedUpdate()
    {
        rig.MovePosition(Vector3.MoveTowards(trans.position, findPath.TargetPoint, Speed));
        Vector3 dir = (findPath.TargetPoint - trans.position).normalized;
        if (dir != Vector3.zero)
            trans.rotation = Quaternion.Lerp(trans.rotation, Quaternion.LookRotation(dir), .1f);
    }
}
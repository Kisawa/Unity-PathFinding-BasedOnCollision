using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(Collider))]
public class PathFinding : MonoBehaviour
{
    static readonly int JitterCount = 24;
    static readonly Vector3 origin = Vector3.zero;

    public List<Collider> Bounds;
    public LayerMask Layer;
    public List<Collider> IgnoreColliders;
    [Range(0, 2)]
    public float HitBias = 0.999f;
    [Range(0.1f, 1)]
    public float StepRange = 1;
    [Range(0, 1)]
    public float HighThreshold = 1;
    public bool OptimizeRoute = true;
    [Range(1, 5)]
    public float RefreshTime = 1;

    public Vector3 TargetPoint { get; private set; }

    public State CurrentState { get; private set; }

    Transform trans;
    new Collider collider;
    Vector3 stepSize;

    Vector3Int stepPoint;
    int jitterRefer;
    Vector3 jitter;

    Collider[] ignoreColliders = new Collider[0];
    BoxCollider boxCollider;
    CapsuleCollider capsuleCollider;
    SphereCollider sphereCollider;
    MeshCollider meshCollider;

    Transform target;
    Vector3Int lastTargetStepPoint;
    Stack<Vector3> Route;
    Coroutine refreshTimer;

    private void Awake()
    {
        trans = transform;
        collider = GetComponent<Collider>();
        if (collider is BoxCollider)
            boxCollider = collider as BoxCollider;
        if (collider is CapsuleCollider)
            capsuleCollider = collider as CapsuleCollider;
        if (collider is SphereCollider)
            sphereCollider = collider as SphereCollider;
        if (collider is MeshCollider)
            meshCollider = collider as MeshCollider;
        refreshStepSize();
    }

    private void FixedUpdate()
    {
        stepPoint = CalcNearestStepPoint(trans.position);
        if (target == null || CurrentState == State.Stop || CurrentState == State.Pause)
        {
            TargetPoint = trans.position;
            return;
        }
        Vector3Int endStepPoint = CalcNearestStepPoint(target.transform.position);
        Stack<Vector3> route = null;
        if (lastTargetStepPoint != endStepPoint)
        {
            jitterRefer = 0;
            Route = null;
            TargetPoint = trans.position;
        }
        if (Route == null && jitterRefer < JitterCount)
        {
            route = FindPathTo(target.position, endStepPoint, ignoreColliders);
            if (route == null)
                refreshJitter();
        }
        if (route != null)
            Route = route;
        if (DistanceTest(TargetPoint))
        {
            if (Route != null && Route.Count > 0)
                TargetPoint = Route.Pop();
            else
                TargetPoint = trans.position;
        }
    }

    public void SetState(State state)
    {
        CurrentState = state;
        if (CurrentState == State.Stop)
            target = null;
        switch (CurrentState)
        {
            case State.Pause:
            case State.Stop:
                jitterRefer = 0;
                TargetPoint = trans.position;
                if (refreshTimer != null)
                {
                    StopCoroutine(refreshTimer);
                    refreshTimer = null;
                }
                Route = null;
                break;
            case State.Start:
                if (refreshTimer == null)
                    refreshTimer = StartCoroutine(refreshProcedure());
                break;
        }
    }

    /// <summary>
    /// auto follow up
    /// </summary>
    public void FindPathTo(Transform target, params Collider[] ignoreCollider)
    {
        this.target = target;
        ignoreColliders = ignoreCollider;
        Route = null;
        SetState(State.Start);
        TargetPoint = trans.position;
    }

    public Stack<Vector3> FindPathTo(Vector3 endPoint, params Collider[] ignoreCollider)
    {
        refreshStepSize();
        ignoreColliders = ignoreCollider;
        Vector3Int endStepPoint = CalcNearestStepPoint(endPoint);
        HashSet<Vector3Int> endNeighborhood = CalcNeighborhood(endStepPoint);
        List<StepInfo> cache = new List<StepInfo>();
        HashSet<Vector3Int> discarded = new HashSet<Vector3Int>();
        discarded.Add(stepPoint);
        return FindPathTo(endPoint, endStepPoint, endNeighborhood, cache, discarded, null);
    }

    Stack<Vector3> FindPathTo(Vector3 endPoint, Vector3Int endStepPoint, params Collider[] ignoreCollider)
    {
        refreshStepSize();
        TargetPoint = trans.position;
        ignoreColliders = ignoreCollider;
        HashSet<Vector3Int> endNeighborhood = CalcNeighborhood(endStepPoint);
        List<StepInfo> cache = new List<StepInfo>();
        HashSet<Vector3Int> discarded = new HashSet<Vector3Int>();
        discarded.Add(stepPoint);
        lastTargetStepPoint = endStepPoint;
        return FindPathTo(endPoint, endStepPoint, endNeighborhood, cache, discarded, null);
    }

    IEnumerator refreshProcedure()
    {
        while (true)
        {
            yield return new WaitForSeconds(RefreshTime);
            switch (CurrentState)
            {
                case State.Start:
                    if (target != null && Route == null && jitterRefer >= JitterCount)
                        jitterRefer = 0;
                    break;
            }
        }
    }

    Stack<Vector3> FindPathTo(Vector3 endPoint, Vector3Int endStepPoint, HashSet<Vector3Int> endNeighborhood, List<StepInfo> cache, HashSet<Vector3Int> discarded, StepInfo current)
    {
        PushNeighborhood(current, endStepPoint, cache, discarded);
        if (cache.Count == 0)
            return null;
        current = CheckMinCostInfo(cache, out int index);
        cache.RemoveAt(index);
        discarded.Add(current.StepPoint);
        if (current.StepPoint == endStepPoint || endNeighborhood.Contains(current.StepPoint))
        {
            Stack<Vector3> route = new Stack<Vector3>();
            if (OptimizeRoute)
            {
                Vector3 prePoint = endPoint;
                prePoint.y = trans.position.y;
                while (current != null)
                {
                    Vector3 _point = current.Point;
                    current = current.Parent;
                    if (route.Count == 0 || HitTest(route.Peek(), _point))
                        route.Push(prePoint);
                    prePoint = _point;
                    if (current == null && (route.Count == 0 || HitTest(route.Peek(), trans.position)))
                        route.Push(prePoint);
                }
            }
            else
            {
                Vector3 _endPoint = endPoint;
                _endPoint.y = trans.position.y;
                route.Push(_endPoint);
                while (current != null)
                {
                    if(current.StepPoint != stepPoint)
                        route.Push(current.Point);
                    current = current.Parent;
                }
            }
            return route;
        }
        return FindPathTo(endPoint, endStepPoint, endNeighborhood, cache, discarded, current);
    }

    StepInfo CheckMinCostInfo(List<StepInfo> infos, out int index)
    {
        StepInfo info = null;
        index = -1;
        for (int i = 0; i < infos.Count; i++)
        {
            StepInfo _info = infos[i];
            if (info == null || info.Cost.x > _info.Cost.x)
            {
                info = _info;
                index = i;
            }
        }
        return info;
    }

    HashSet<Vector3Int> CalcNeighborhood(Vector3Int stepPoint)
    {
        HashSet<Vector3Int> Neighborhood = new HashSet<Vector3Int>();
        Vector3Int forwardSeek = stepPoint + new Vector3Int(0, 0, 1);
        if (!HitTest(CalcPoint(forwardSeek, jitter)))
            Neighborhood.Add(forwardSeek);
        Vector3Int backSeek = stepPoint + new Vector3Int(0, 0, -1);
        if (!HitTest(CalcPoint(backSeek, jitter)))
            Neighborhood.Add(backSeek);
        Vector3Int rightSeek = stepPoint + new Vector3Int(1, 0, 0);
        if (!HitTest(CalcPoint(rightSeek, jitter)))
            Neighborhood.Add(rightSeek);
        Vector3Int leftSeek = stepPoint + new Vector3Int(-1, 0, 0);
        if (!HitTest(CalcPoint(leftSeek, jitter)))
            Neighborhood.Add(leftSeek);
        Vector3Int forwardRightSeek = stepPoint + new Vector3Int(1, 0, 1);
        if (!HitTest(CalcPoint(forwardRightSeek, jitter)))
            Neighborhood.Add(forwardRightSeek);
        Vector3Int forwardLeftSeek = stepPoint + new Vector3Int(-1, 0, 1);
        if (!HitTest(CalcPoint(forwardLeftSeek, jitter)))
            Neighborhood.Add(forwardLeftSeek);
        Vector3Int backRightSeek = stepPoint + new Vector3Int(1, 0, -1);
        if (!HitTest(CalcPoint(backRightSeek, jitter)))
            Neighborhood.Add(backRightSeek);
        Vector3Int backLeftSeek = stepPoint + new Vector3Int(-1, 0, -1);
        if (!HitTest(CalcPoint(backLeftSeek, jitter)))
            Neighborhood.Add(backLeftSeek);
        return Neighborhood;
    }

    void PushNeighborhood(StepInfo current, Vector3Int endStepPoint, List<StepInfo> cache, HashSet<Vector3Int> discarded)
    {
        Vector3Int stepPoint = current == null ? this.stepPoint : current.StepPoint;
        Vector3Int forwardSeek = stepPoint + new Vector3Int(0, 0, 1);
        PushSetpInfo(forwardSeek, current, endStepPoint, cache, discarded);
        Vector3Int backSeek = stepPoint - new Vector3Int(0, 0, 1);
        PushSetpInfo(backSeek, current, endStepPoint, cache, discarded);
        Vector3Int rightSeek = stepPoint + new Vector3Int(1, 0, 0);
        PushSetpInfo(rightSeek, current, endStepPoint, cache, discarded);
        Vector3Int leftSeek = stepPoint - new Vector3Int(1, 0, 0);
        PushSetpInfo(leftSeek, current, endStepPoint, cache, discarded);
        Vector3Int forwardRightSeek = stepPoint + new Vector3Int(1, 0, 1);
        PushSetpInfo(forwardRightSeek, current, endStepPoint, cache, discarded);
        Vector3Int forwardLeftSeek = stepPoint + new Vector3Int(-1, 0, 1);
        PushSetpInfo(forwardLeftSeek, current, endStepPoint, cache, discarded);
        Vector3Int backRightSeek = stepPoint + new Vector3Int(1, 0, -1);
        PushSetpInfo(backRightSeek, current, endStepPoint, cache, discarded);
        Vector3Int backLeftSeek = stepPoint + new Vector3Int(-1, 0, -1);
        PushSetpInfo(backLeftSeek, current, endStepPoint, cache, discarded);
    }

    void PushSetpInfo(Vector3Int stepPoint, StepInfo parent, Vector3Int endStepPoint, List<StepInfo> cache, HashSet<Vector3Int> discarded)
    {
        Vector3 point = CalcPoint(stepPoint, jitter);
        if (!ContainsBounds(point) || discarded.Contains(stepPoint))
            return;
        StepInfo _info = cache.FirstOrDefault(x => x.StepPoint == stepPoint);
        StepInfo.PreView(this, parent, stepPoint, endStepPoint, out Vector2Int G_cost, out Vector2Int H_cost);
        Vector2Int cost = G_cost + H_cost;
        if (_info == null)
        {
            Vector3 parentPoint = parent == null ? trans.position : parent.Point;
            if (cost.y == 0 && !HitTest(point, parentPoint))
                cache.Add(new StepInfo(this, parent, point, stepPoint, G_cost, H_cost));
        }
        else
        {
            if (cost.x <= _info.Cost.x)
                _info.Refresh(parent, G_cost, H_cost);
        }
    }

    bool ContainsBounds(Vector3 point)
    {
        bool res = false;
        for (int i = 0; i < Bounds.Count; i++)
        {
            if (Bounds[i].bounds.Contains(point))
                res = true;
        }
        return res;
    }

    void refreshJitter()
    {
        jitterRefer = ++jitterRefer > JitterCount ? 0 : jitterRefer;
        jitter = new Vector3(HaltonSeq(2, jitterRefer + 1), 0, HaltonSeq(3, jitterRefer + 1));
        jitter.x *= stepSize.x;
        jitter.z *= stepSize.z;
    }

    float HaltonSeq(int refer, int index = 1/* NOT! zero-based */)
    {
        float result = 0;
        float fraction = 1;
        int i = index;
        while (i > 0)
        {
            fraction /= refer;
            result += fraction * (i % refer);
            i = (int)Mathf.Floor(i / (float)refer);
        }
        return result;
    }

    Vector2Int CalcCost(Vector3Int from, Vector3Int to)
    {
        Vector3Int DIF = to - from;
        //plane-x-z
        int abs_x = Mathf.Abs(DIF.x);
        int abs_z = Mathf.Abs(DIF.z);
        int cost = abs_x < abs_z ? abs_x * 14 + (abs_z - abs_x) * 10 : abs_z * 14 + (abs_x - abs_z) * 10;
        return new Vector2Int(cost, DIF.y);
    }

    Vector3Int CalcStepPoint(Vector3 point)
    {
        Vector3 _origin = origin;
        _origin.y = trans.position.y;
        Vector3 local = point - _origin;
        Vector3Int stepPoint = Vector3Int.zero;
        stepPoint.x = (int)(local.x / stepSize.x);
        stepPoint.y = (int)(local.y / stepSize.y);
        stepPoint.z = (int)(local.z / stepSize.z);
        return stepPoint;
    }

    Vector3 CalcPoint(Vector3Int stepPoint, Vector3 offset)
    {
        Vector3 _origin = origin;
        _origin.y = trans.position.y;
        Vector3 point = _origin + Vector3.forward * stepSize.z * stepPoint.z;
        point += Vector3.right * stepSize.x * stepPoint.x;
        point += offset;
        return point;
    }

    Vector3Int CalcNearestStepPoint(Vector3 point)
    {
        Vector3Int center = CalcStepPoint(point);
        float dis = Vector3.Distance(point, CalcPoint(center, Vector3.zero));
        Vector3Int stepPoint = center;
        Vector3Int forwardSeek = center + new Vector3Int(0, 0, 1);
        float forwardDis = Vector3.Distance(point, CalcPoint(forwardSeek, Vector3.zero));
        if (forwardDis < dis && ContainsBounds(forwardSeek))
        {
            dis = forwardDis;
            stepPoint = forwardSeek;
        }
        Vector3Int backSeek = center + new Vector3Int(0, 0, -1);
        float backDis = Vector3.Distance(point, CalcPoint(backSeek, Vector3.zero));
        if (backDis < dis && ContainsBounds(backSeek))
        {
            dis = backDis;
            stepPoint = backSeek;
        }
        Vector3Int rightSeek = center + new Vector3Int(1, 0, 0);
        float rightDis = Vector3.Distance(point, CalcPoint(rightSeek, Vector3.zero));
        if (rightDis < dis && ContainsBounds(rightSeek))
        {
            dis = rightDis;
            stepPoint = rightSeek;
        }
        Vector3Int leftSeek = center + new Vector3Int(-1, 0, 0);
        float leftDis = Vector3.Distance(point, CalcPoint(leftSeek, Vector3.zero));
        if (leftDis < dis && ContainsBounds(leftSeek))
        {
            dis = leftDis;
            stepPoint = leftSeek;
        }
        Vector3Int forwardRightSeek = center + new Vector3Int(1, 0, 1);
        float forwardRightDis = Vector3.Distance(point, CalcPoint(forwardRightSeek, Vector3.zero));
        if (forwardRightDis < dis && ContainsBounds(forwardRightSeek))
        {
            dis = forwardRightDis;
            stepPoint = forwardRightSeek;
        }
        Vector3Int forwardLeftSeek = center + new Vector3Int(-1, 0, 1);
        float forwardLeftDis = Vector3.Distance(point, CalcPoint(forwardLeftSeek, Vector3.zero));
        if (forwardLeftDis < dis && ContainsBounds(forwardLeftSeek))
        {
            dis = forwardLeftDis;
            stepPoint = forwardLeftSeek;
        }
        Vector3Int backRightSeek = center + new Vector3Int(1, 0, -1);
        float backRightDis = Vector3.Distance(point, CalcPoint(backRightSeek, Vector3.zero));
        if (backRightDis < dis && ContainsBounds(backRightSeek))
        {
            dis = backRightDis;
            stepPoint = backRightSeek;
        }
        Vector3Int backLeftSeek = center + new Vector3Int(-1, 0, -1);
        float backLeftDis = Vector3.Distance(point, CalcPoint(backLeftSeek, Vector3.zero));
        if (backLeftDis < dis && ContainsBounds(backLeftSeek))
            stepPoint = backLeftSeek;
        return stepPoint;
    }

    //  single point hit
    bool HitTest(Vector3 point)
    {
        return hitTest(point, Layer);
    }

    bool HitTest(Vector3 point0, Vector3 point1)
    {
        return hitTest(point0, point1, Layer);
    }

    bool hitTest(Vector3 point, int layer)
    {
        Collider[] colliders = new Collider[IgnoreColliders.Count + ignoreColliders.Length + 2];
        int count = 0;
        if (boxCollider != null || meshCollider != null)
            count = Physics.OverlapBoxNonAlloc(point, collider.bounds.extents * HitBias, colliders, trans.rotation, layer);
        else if (sphereCollider != null)
            count = Physics.OverlapSphereNonAlloc(point, sphereCollider.radius * HitBias, colliders, layer);
        else if (capsuleCollider != null)
        {
            Vector3 dir = Vector3.zero;
            switch (capsuleCollider.direction)
            {
                case 0:
                    dir = trans.right;
                    break;
                case 1:
                    dir = trans.up;
                    break;
                case 2:
                    dir = trans.forward;
                    break;
            }
            Vector3 point0 = point + dir * (capsuleCollider.height * .5f * HitBias - capsuleCollider.radius);
            Vector3 point1 = point - dir * (capsuleCollider.height * .5f * HitBias - capsuleCollider.radius);
            count = Physics.OverlapCapsuleNonAlloc(point0, point1, capsuleCollider.radius * HitBias, colliders, layer);
        }
        for (int i = 0; i < count; i++)
        {
            Collider _collider = colliders[i];
            if (_collider != collider && !IgnoreColliders.Contains(_collider) && Array.IndexOf(ignoreColliders, _collider) < 0)
                return true;
        }
        return false;
    }

    bool hitTest(Vector3 point0, Vector3 point1, int layer, List<Collider> _colliders = null)
    {
        if (_colliders != null)
            _colliders.Clear();
        Collider[] colliders = new Collider[IgnoreColliders.Count + ignoreColliders.Length + 2];
        int count = 0;
        if (boxCollider != null || meshCollider != null)
        {
            Vector3 dir = Vector3.Normalize(point1 - point0);
            float dis = Vector3.Distance(point0, point1);
            Vector3 point = point0 + dir * dis * 0.5f;
            Bounds bound = collider.bounds;
            bound.center = point0;
            bound.Encapsulate(point1);
            count = Physics.OverlapBoxNonAlloc(point, bound.extents * HitBias, colliders, Quaternion.identity, layer);
        }
        else if (sphereCollider != null)
            count = Physics.OverlapCapsuleNonAlloc(point0, point1, sphereCollider.radius * HitBias, colliders, layer);
        else if (capsuleCollider != null)
            count = Physics.OverlapCapsuleNonAlloc(point0, point1, capsuleCollider.radius * HitBias, colliders, layer);
        bool res = false;
        for (int i = 0; i < count; i++)
        {
            Collider _collider = colliders[i];
            if (_collider != collider && !IgnoreColliders.Contains(_collider) && Array.IndexOf(ignoreColliders, _collider) < 0)
            {
                res = true;
                if (_colliders != null)
                    _colliders.Add(_collider);
            }
        }
        return res;
    }

    bool DistanceTest(Vector3 point, float threshold = 0.05f)
    {
        return Vector3.Distance(point, trans.position) < threshold;
    }

    void refreshStepSize()
    {
        Vector3 size;
        if (boxCollider != null)
            size = boxCollider.size;
        else if (sphereCollider != null)
        {
            float radius = sphereCollider.radius;
            size = new Vector3(radius * 2, radius * 2, radius * 2);
        }
        else if (capsuleCollider)
        {
            float radius = capsuleCollider.radius;
            float height = capsuleCollider.height;
            int direction = capsuleCollider.direction;
            Vector3[] directionArray = new Vector3[] { Vector3.right, Vector3.up, Vector3.forward };
            Vector3 result = new Vector3();
            for (int i = 0; i < 3; i++)
            {
                if (i == direction)
                    result += directionArray[i] * height;
                else
                    result += directionArray[i] * radius * 2;
            }
            size = result;
        }
        else
            size = meshCollider.sharedMesh.bounds.size;
        Vector3 scale = trans.lossyScale;
        stepSize = new Vector3(size.x * scale.x, size.y * scale.y, size.z * scale.z);
        stepSize.x *= StepRange;
        stepSize.z *= StepRange;
        stepSize.y *= HighThreshold;
    }

    class StepInfo
    {
        public StepInfo Parent { get; private set; }
        public Vector3 Point { get; private set; }

        public Vector3Int StepPoint { get; private set; }

        public Vector2Int G_Cost { get; private set; }

        public Vector2Int H_Cost { get; private set; }

        public Vector2Int Cost { get; private set; }

        PathFinding core;

        public static void PreView(PathFinding core, StepInfo parent, Vector3Int stepPoint, Vector3Int endStepPoint, out Vector2Int G_cost, out Vector2Int H_cost)
        {
            Vector3Int parentStepPoint = parent == null ? core.stepPoint : parent.StepPoint;
            Vector2Int parentG_Cost = parent == null ? Vector2Int.zero : parent.G_Cost;
            G_cost = core.CalcCost(parentStepPoint, stepPoint) + parentG_Cost;
            H_cost = core.CalcCost(stepPoint, endStepPoint);
        }

        public StepInfo(PathFinding core, StepInfo parent, Vector3 point, Vector3Int stepPoint, Vector2Int G_cost, Vector2Int H_cost)
        {
            this.core = core;
            Parent = parent;
            Point = point;
            StepPoint = stepPoint;
            G_Cost = G_cost;
            H_Cost = H_cost;
            Cost = G_Cost + H_Cost;
        }

        public Vector2Int RefreshCost(StepInfo parent, Vector3Int endStepPoint)
        {
            CalcCost(parent, endStepPoint);
            return Cost;
        }

        public void Refresh(StepInfo parent, Vector2Int G_cost, Vector2Int H_cost)
        {
            Parent = parent;
            G_Cost = G_cost;
            H_Cost = H_cost;
            Cost = G_Cost + H_Cost;
        }

        void CalcCost(StepInfo parent, Vector3Int endStepPoint)
        {
            Vector3Int parentStepPoint = parent == null ? core.stepPoint : parent.StepPoint;
            Vector2Int parentG_Cost = parent == null ? Vector2Int.zero : parent.G_Cost;
            G_Cost = core.CalcCost(parentStepPoint, StepPoint) + parentG_Cost;
            H_Cost = core.CalcCost(StepPoint, endStepPoint);
            Cost = G_Cost + H_Cost;
        }
    }

    public enum State
    {
        Stop,
        Start,
        Pause
    }
}
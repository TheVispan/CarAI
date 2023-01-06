using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class AI : MonoBehaviour
{
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float MaxSpeed;
    public float maxSteeringAngle;
    public float maxBrake;
    public Transform COM;
    public List<GameObject> Waypoints = new List<GameObject>();
    public float MaxDistance;
    public HingeJoint Harvester;
    public float SpeedAgent;
    public float MaxDistanceAgent;
    
    private int CurrentWP;
    private Transform Agent;
    private bool initialized;

    void Start()
    {
        GetComponent<Rigidbody>().centerOfMass = COM.localPosition;
        GameObject agent = new GameObject();
        agent.name = "NavMeshAgent";
        agent.AddComponent<NavMeshAgent>();
        agent.transform.localPosition = transform.position;
        agent.tag = "Target";
        initialized = true;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;

        Waypoints = new List<GameObject>(GameObject.FindGameObjectsWithTag("Waypoint"));
        for (int i = 0; i < Waypoints.Count - 1; i++)
        {
            Gizmos.DrawLine(Waypoints[i].transform.position, Waypoints[i + 1].transform.position);
            Gizmos.DrawSphere(Waypoints[i].transform.position, 0.5f);
            Gizmos.DrawSphere(Waypoints[i + 1].transform.position, 0.5f);
        }
        if (initialized == true)
        {
            Agent = GameObject.FindGameObjectWithTag("Target").transform;
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(Agent.position, 1);
        }
    }

    void CarAI()
    {
        float motor = 0f;
        float steering = 0f;
        float angleAccel;
        float angleRotate;

        Agent = GameObject.FindGameObjectWithTag("Target").transform;
        Waypoints = new List<GameObject>(GameObject.FindGameObjectsWithTag("Waypoint"));
        if (1.0f > Vector3.Distance(Agent.position, Waypoints[CurrentWP].transform.position))
        {
            CurrentWP++;
            if (CurrentWP >= Waypoints.Count)
            {
                CurrentWP = 0;
            }
        }
        Agent.GetComponent<NavMeshAgent>().SetDestination(Waypoints[CurrentWP].transform.position);

        if (Agent != null)
        {
            angleAccel = -(Vector3.Angle(Agent.position - transform.position, transform.forward) - 90);
            if (MaxDistanceAgent < Vector3.Distance(transform.position, Agent.position))
            {
                Agent.GetComponent<NavMeshAgent>().speed = 0;
            }
            else
            {
                Agent.GetComponent<NavMeshAgent>().speed = SpeedAgent;
            }
            motor = Mathf.Clamp(angleAccel, -1, 1);

            if (motor > 0)
            {
                angleRotate = -(Vector3.Angle(Agent.position - transform.position, transform.right) - 90);
            }
            else
            {
                angleRotate = maxSteeringAngle;
            }
            steering = Mathf.Clamp(angleRotate, -maxSteeringAngle, maxSteeringAngle);

            if (MaxDistance > Vector3.Distance(transform.position, Agent.position))
            {
                motor = 0;
            }
        }
        CarMove(motor, steering);
    }

    void FixedUpdate()
    {
        CarAI();
        HarvesterMove();
        UpdateWheels();
    }

    public void HarvesterMove()
    {
        var HarvesterMove = Harvester.motor;
        HarvesterMove.force = 100.0f;
        HarvesterMove.targetVelocity = 150;
        Harvester.motor = HarvesterMove;
        Harvester.useMotor = true;
    }

    void CarMove(float motor, float steering)
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (motor == 0)
            {
                axleInfo.rightWheel.brakeTorque = maxBrake;
                axleInfo.leftWheel.brakeTorque = maxBrake;
                
            } 
            else
            {
                axleInfo.rightWheel.brakeTorque = 0;
                axleInfo.leftWheel.brakeTorque = 0;
                if (axleInfo.leftWheel.rpm > MaxSpeed)
                {
                    axleInfo.leftWheel.motorTorque = 0;
                }
                else
                {
                    axleInfo.leftWheel.motorTorque = motor * maxMotorTorque;
                }

                if (axleInfo.rightWheel.rpm > MaxSpeed)
                {
                    axleInfo.rightWheel.motorTorque = 0;
                }
                else
                {
                    axleInfo.rightWheel.motorTorque = motor * maxMotorTorque;
                }
            }
 
        }
    }

    void UpdateWheels()
    {
        foreach (AxleInfo axleInfo in axleInfos)
        {
            axleInfo.leftWheelGrahic.transform.Rotate(Mathf.Repeat(Time.fixedDeltaTime * -axleInfo.leftWheel.rpm * 360.0f / 60.0f, 360.0f), 0, 0);
            axleInfo.rightWheelGraphic.transform.Rotate(Mathf.Repeat(Time.fixedDeltaTime * -axleInfo.rightWheel.rpm * 360.0f / 60.0f, 360.0f), 0, 0);
        }
    }

    [System.Serializable]
    public class AxleInfo
    {
        public WheelCollider leftWheel;
        public Transform leftWheelGrahic;
        public WheelCollider rightWheel;
        public Transform rightWheelGraphic;

        public bool motor;
        public bool steering;

    }
}

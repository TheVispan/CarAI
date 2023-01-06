using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Collecting : MonoBehaviour
{   
    public Mesh CollectingMesh;
    private bool collecting;
    private void OnTriggerStay(Collider other)
    {
        if (other.tag == "Harvester")
        {
            collecting = true;
        }
        
    }

    void FixedUpdate()
    {
        if (collecting == true)
        {
            GetComponent<MeshFilter>().mesh = CollectingMesh;
        }
    }
}

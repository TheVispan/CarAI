using UnityEngine;
using UnityEngine.AI;
using static UnityEngine.GraphicsBuffer;

namespace Eccentric {

    public class CameraMove : MonoBehaviour {


        [SerializeField]  private float _mouseSensitivity = 0.4f;
        [SerializeField] private float _moveSpeed = 2f;
        [SerializeField] private GameObject Object;
        private NavMeshAgent agent;
        private GameObject obj, last_obj;
        private Vector3 _mousePreveousePos;
        private float _rotationX;
        private float _rotationY;



        void Update() {
            if (agent == null)
            {
                agent = GameObject.FindGameObjectWithTag("Target").GetComponent<NavMeshAgent>();
            }
            Move();
            Rotate();
            SpawnTarget();
        }

        void Move() {

            float shiftMult = 1f;
            if (Input.GetKey(KeyCode.LeftShift)) {
                shiftMult = 3f;
            }

            float right = Input.GetAxis("Horizontal");
            float forward = Input.GetAxis("Vertical");
            float up = 0;


            Vector3 offset = new Vector3(right, up, forward) * _moveSpeed * shiftMult * Time.unscaledDeltaTime;
            transform.Translate(offset);
        }

        void Rotate() {

            Vector3 _mouseDelta;

            if (Input.GetMouseButtonDown(1)) {
                _mousePreveousePos = Input.mousePosition;
            }

            if (Input.GetMouseButton(1)) {
                _mouseDelta = Input.mousePosition - _mousePreveousePos;
                _mousePreveousePos = Input.mousePosition;

                _rotationX -= _mouseDelta.y * _mouseSensitivity;
                _rotationY += _mouseDelta.x * _mouseSensitivity;

                transform.localEulerAngles = new Vector3(_rotationX, _rotationY, 0f);
            }
        }

        void SpawnTarget()
        {
            
            if (Input.GetMouseButtonDown(0))
            {
                
                //SpawnObject.SetActive(!SpawnObject.activeSelf);
                /*
                RaycastHit hit;
                if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hit))
                {
                    GameObject obj = Instantiate(Object, new Vector3(hit.point.x, hit.point.y + 2, hit.point.z), Quaternion.identity) as GameObject;
                    agent.SetDestination(hit.point);
                    var new_obj = obj;
                    DestroyImmediate(last_obj);
                    obj.transform.rotation = Quaternion.Euler(-90, 0, 0);
                    last_obj = new_obj;

                }
                */
            }
        }

    }

}


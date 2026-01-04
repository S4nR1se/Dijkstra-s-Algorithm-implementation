using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// _     __________    _    ______   __   ____ ___  ____  _____           
//| |   | ____/ ___|  / \  / ___\ \ / /  / ___/ _ \|  _ \| ____|          
//| |   |  _|| |  _  / _ \| |    \ V /  | |  | | | | | | |  _|            
//| |___| |__| |_| |/ ___ | |___  | |   | |__| |_| | |_| | |___           
//|_____|_____\____/_/   \_\____| |_|    \____\___/|____/|_____|          


// ____   ___    _   _  ___ _____    ____ _   _    _    _   _  ____ _____ 
//|  _ \ / _ \  | \ | |/ _ |_   _|  / ___| | | |  / \  | \ | |/ ___| ____|
//| | | | | | | |  \| | | | || |   | |   | |_| | / _ \ |  \| | |  _|  _|  
//| |_| | |_| | | |\  | |_| || |   | |___|  _  |/ ___ \| |\  | |_| | |___ 
//|____/ \___/  |_| \_|\___/ |_|    \____|_| |_/_/   \_|_| \_|\____|_____|
public class Burger : MonoBehaviour
{
    [SerializeField] AnimationCurve curve;

    Vector3 StartPos = Vector3.zero;
    Vector3 EndPos = new Vector3(0, 0.5f, 0);

    private void Awake()
    {
        StartPos = transform.position;
        EndPos = StartPos + EndPos;
    }

    private void Update()
    {
        transform.position = Vector3.Lerp(StartPos, EndPos, curve.Evaluate(Mathf.PingPong(Time.time, 1)));

        transform.Rotate(Vector3.up * 90 * Time.deltaTime);
    }
}

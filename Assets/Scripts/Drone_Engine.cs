﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VRLab {

    public class Drone_Engine : MonoBehaviour, IEngine
    {
        #region Variables
        [Header("Engine Properties")]
        [SerializeField] private float maxPower = 4f;
        #endregion
        #region Interface Methods
        public void InitEngine()
        {
            throw new System.NotImplementedException();
        }

        public void UpdateEngine(Rigidbody rb, Drone_Inputs input)
        {
            Vector3 engineForce = Vector3.zero;
            //adds more hover power up or going down and we want divide by number of engines we have
            engineForce = transform.up * ((rb.mass * Physics.gravity.magnitude) + (input.Throttle * maxPower)) / 8f;

            rb.AddForce(engineForce, ForceMode.Force);
        }
        #endregion
    }
}

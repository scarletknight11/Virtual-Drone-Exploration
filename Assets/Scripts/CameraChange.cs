using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraChange : MonoBehaviour
{
    public GameObject cam1;
    public GameObject cam2;
    public int CamMode;


    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.L)) {
             if (CamMode == 1) {
                 CamMode = 0;
            } else {
                CamMode += 1;
            }
            StartCoroutine(CamChange());
        }  
    }

    IEnumerator CamChange()
    {
        yield return new WaitForSeconds(0.01f);
        if (CamMode == 0)
        {
            cam1.SetActive(true);
            cam2.SetActive(false);
        }
        if (CamMode == 1)
        {
            cam1.SetActive(false);
            cam2.SetActive(true);
        }
    }

}

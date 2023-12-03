using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine.UIElements;
using TMPro;
using System;

public class BLE : MonoBehaviour
{

    Thread thread;
    public int connectionPort;
    TcpListener server;
    TcpClient client;
    bool running;
    Vector3 ballRotation = Vector3.zero;
    Vector3 accelRotation = Vector3.zero;
    Vector3 gravityRotation = Vector3.zero;
    Vector3 cameraRotation = Vector3.zero;
    double time_ms;
    double deg; 
    int t;
    //Vector3 currentEuler = Vector3.zero;
    public GameObject ball;
    public GameObject accelPointer;
    public GameObject gravityPointer;
    public GameObject pointingCamera;
    public TextMeshProUGUI gui;
    public GameObject trackerRotaterPrev;
    public GameObject trackerRotaterCurr;
    public GameObject trackerLocPrev;
    public GameObject trackerLocCurr;
    bool rotated = false;
    float accumTime = 0;
    float prevTime;
    float currTime;
    public static string[] SplitBySemiColon(string data)
    {
        return data.Split(';');
    }
    public static Vector3 ParseData(string data)
    {

        if (data.StartsWith("(") && data.EndsWith(")"))
        {
            data = data.Substring(1, data.Length - 2);

        }

        string[] strArr = data.Split(',');

        Vector3 res = new Vector3(
            float.Parse(strArr[0]),
            float.Parse(strArr[1]),
            float.Parse(strArr[2]));
        return res;
    }

    void Connection()
    {
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);

        //string of data received
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead);
        if (dataReceived != null && dataReceived != "")
        {
            //extract based on index after split
            //rotation = ParseData(dataReceived);
            string[] current = SplitBySemiColon(dataReceived);
            ballRotation = ParseData(current[0]);
            accelRotation = ParseData(current[1]);
            gravityRotation = ParseData(current[2]);
            cameraRotation = ParseData(current[3]);
            time_ms = Convert.ToDouble(current[4]);
            deg = Convert.ToDouble(current[5]);
            nwStream.Write(buffer, 0, bytesRead);
        }
    }

    void GetData() {
        server = new TcpListener(IPAddress.Any, connectionPort);
        server.Start();
        client = server.AcceptTcpClient();
        running= true;
        Debug.Log("Started server connection at port: " + connectionPort);
        while (running)
        {
            Connection();
        }
        server.Stop();
    }

    // Start is called before the first frame update
    void Start()
    {
        ThreadStart ts = new ThreadStart(GetData);
        thread = new Thread(ts);
        thread.Start();

    }

    // Update is called once per frame
    void Update()
    {
        currTime = Time.time;
        ball.transform.rotation = Quaternion.Euler(ballRotation);
        // rotate current orientation relative to ball
        trackerRotaterCurr.transform.rotation = Quaternion.Euler(ballRotation);
        accelPointer.transform.rotation = Quaternion.Euler(accelRotation);
        gravityPointer.transform.rotation = Quaternion.Euler(gravityRotation);
        pointingCamera.transform.rotation = Quaternion.Euler(cameraRotation);
        if (!rotated)
        {
            // first time rotated, rotate previous to current
            //trackerRotaterPrev.transform.rotation = Quaternion.Euler(ballRotation);
            prevTime= Time.time;
            accumTime = 0;
            rotated= true;
            gui.SetText("RPM: 0");
        }
        else
        {
            if (accumTime+(currTime - prevTime) > 0.1)
            {
                // get previous and current position of trackers
                // find angle subtended using cos product and hence RPM using time period
                Vector3 prevPosition = trackerLocPrev.transform.position;
                Vector3 currPosition = trackerLocCurr.transform.position;
                double dotProduct = prevPosition[0] * currPosition[0] + prevPosition[1] * currPosition[1] + prevPosition[2] * currPosition[2];
                double magPrev = Math.Pow(Math.Pow(prevPosition[0], 2) + Math.Pow(prevPosition[1], 2) + Math.Pow(prevPosition[2], 2), 0.5);
                double magCurr = Math.Pow(Math.Pow(currPosition[0], 2) + Math.Pow(currPosition[1], 2) + Math.Pow(currPosition[2], 2), 0.5);
                // angle subtended in degrees
                double angleSubtended = Math.Acos(dotProduct/(magPrev*magCurr))*(180/Math.PI);
                // find degree rotation per second, multiply by 60 for degree rotation per minute, divide by 360 for rotation per minute
                double RPM = angleSubtended/(accumTime*6);
                //Debug.Log(accumTime +","+RPM +","+angleSubtended+","+(prevPosition[0]-currPosition[0]) + "," + (prevPosition[1] - currPosition[1]) + "," + (prevPosition[2] - currPosition[2]));
                Debug.Log(angleSubtended);
                gui.SetText("RPM: " + RPM.ToString("F0")+"\n"+ "Angle: "+ angleSubtended.ToString("F0") + "\n"+"aX: "+ accelRotation[0]+"\n" + "aY: " + accelRotation[1] + "\n" + "bX: " + gravityRotation[0] + "\n" + "bY: " + gravityRotation[1] + "\n" + "Deg: " + deg + "\n");
                // rotate previos tracker to track current location
                trackerRotaterPrev.transform.rotation = Quaternion.Euler(ballRotation);
                accumTime= 0;
                prevTime = Time.time;

            } else
            {
                //trackerRotaterPrev.transform.rotation = Quaternion.Euler(ballRotation);
                accumTime += currTime-prevTime;
                prevTime=Time.time;
            }
        }
        
    }
}

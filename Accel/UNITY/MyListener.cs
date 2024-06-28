using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;

public class MyListener : MonoBehaviour
{
    Thread thread;
    public int connectionPort = 25001;
    TcpListener server;
    TcpClient client;
    bool running;

    void Start()
    {
        // Receive on a separate thread so Unity doesn't freeze waiting for data
        ThreadStart ts = new ThreadStart(GetData);
        thread = new Thread(ts);
        thread.Start();
    }

    void GetData()
    {
        // Create the server
        server = new TcpListener(IPAddress.Any, connectionPort);
        server.Start();

        // Create a client to get the data stream
        client = server.AcceptTcpClient();

        // Start listening
        running = true;
        while (running)
        {
            Connection();
        }
        server.Stop();
    }

    void Connection()
    {
        // Read data from the network stream
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize);

        // Decode the bytes into a string
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead);

        // Make sure we're not getting an empty string
        if (!string.IsNullOrEmpty(dataReceived))
        {
            // Convert the received string of data to the format we are using
            Vector3 position;
            Vector3 rotation;
            ParseData(dataReceived, out position, out rotation);
            nwStream.Write(buffer, 0, bytesRead);
            // Set the position and rotation of the object
            UpdateTransform(position, rotation);
        }
    }

    // Use-case specific function, need to re-write this to interpret whatever data is being sent
    public static void ParseData(string dataString, out Vector3 position, out Vector3 rotation)
    {
        // Split the elements into an array
        string[] stringArray = dataString.Split(',');

        // Store position as a Vector3
        position = new Vector3(
            float.Parse(stringArray[0]),
            float.Parse(stringArray[2]),
            float.Parse(stringArray[1]));

        // Store rotation as a Vector3
        rotation = new Vector3(
            float.Parse(stringArray[3]),
            float.Parse(stringArray[5]),
            float.Parse(stringArray[4]));
    }

    void UpdateTransform(Vector3 newPosition, Vector3 newRotation)
    {
        // Set this object's position in the scene according to the position received
        position = newPosition;
        rotation = newRotation;
    }

    // Position is the data being received in this example
    Vector3 position = Vector3.zero;
    Vector3 rotation = Vector3.zero;

    void Update()
    {
        // Update the transform's position on the main thread
        transform.position = position;
        transform.eulerAngles = rotation;
    }

    void OnApplicationQuit()
    {
        running = false;
        thread.Abort();
        client.Close();
    }
}

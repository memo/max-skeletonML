import com.cycling74.max.*;
import com.cycling74.jitter.*;
import java.util.*;

public class Kinect2Receiver extends MaxObject
{
	private final int version = 79;			// version of code. useful to see if patch is using correct version or not!
	private final String usersMatName = "usersMatrix";

	private int numUsers = 10;		// matrix planes
	private int numJoints = 25;		// matrix rows
	private int numFeatures = 11;	// matrix columns: confidence posx posy posz quatx quaty quatz quatw yaw pitch roll
	private JitterMatrix usersMat;	// matrix storing user info

	private int overrideUserId = -1;// if this is non-negative, overrider userId with this (useful for debugging)

	private Map<String, Integer> jointNameToIndex = new HashMap<String, Integer>();	// map joint name to index

	private boolean verbose = false;



	// some temp variables to save from reallocating every frame
	private float jointFeatures[];			// joint features for one joint
	private float quat[] = new float[4];	// joint quat, to be converted to euler
	private float euler[] = new float[3];	// joint euler, convereted from quat

	public Kinect2Receiver(Atom[] args)	{
		// declare IO ports
		declareIO(2, 2);

		// set IO port tooltips
		setInletAssist(0, "messages from Kinect2Sender");
		setInletAssist(1, "joint tuples [name index]");
		setOutletAssist(0, "jit.matrix with user data");
		setOutletAssist(1, "commands to set jit.cellblock with joint names");

		// declare attributes
		declareAttribute("numUsers", null, "setnumUsers");
		declareAttribute("numJoints", null, "setnumJoints");
		declareAttribute("numFeatures", null, "setnumFeatures");
		declareAttribute("verbose");
		declareAttribute("overrideUserId");

		// dump version to console
		dumpVersion();

		// init
		initMatrix();
	}

	private void initMatrix() {
		post("Kinect2Receiver::initMatrix - " + numUsers + " users x " + numJoints + " joints x " + numFeatures + " features");
		usersMat = new JitterMatrix(usersMatName, numUsers, "float32", numFeatures, numJoints);
		jointFeatures = new float[numFeatures];
	}

	private void setnumUsers(int n) {
		if(verbose) post("Kinect2Receiver::setnumUsers - " + n);
		numUsers = n;
		initMatrix();
	}

	private void setnumJoints(int n) {
		if(verbose) post("Kinect2Receiver::setnumJoints - " + n);
		numJoints = n;
		initMatrix();
	}

	private void setnumFeatures(int n) {
		if(verbose) post("Kinect2Receiver::setnumFeatures - " + n);
		numFeatures = n;
		initMatrix();
	}

	public void loadbang() {
		post("Kinect2Receiver::loadbang");
	}
    
	public void bang() {
		outlet(0, "jit_matrix", usersMat.getName());
	}
    
	public void inlet(int i) {
	}
    
	public void inlet(float f) {
	}
    
	public void list(Atom[] l) {
		if(verbose) post("Kinect2Receiver::list - " + Atom.toOneString(l));	// convert array to string and dump to console
	}

	public void anything(String s, Atom[] args) {
		if(verbose) post("Kinect2Receiver::anything - " + s + ": " + Atom.toOneString(args));	// convert array to string and dump to console
		if(getInlet() == 0) {
			if(s.equals("skel")) handleSkeletonFeatures(args);
		} else if(getInlet() == 1) {
			handleJointName(s, args);
		}
	}

	private void handleJointName(String jointName, Atom[] args) {
		post("Kinect2Receiver::handleJointName - " + jointName + ": " + Atom.toOneString(args));
		int jointIndex = args[0].getInt();
		jointNameToIndex.put(jointName, jointIndex);
		outlet(1, "set", new Atom[] { Atom.newAtom(0), Atom.newAtom(jointIndex), Atom.newAtom(jointIndex + ": " + jointName) } );
	}

	private void handleSkeletonFeatures(Atom[] args) {
		int userId = overrideUserId < 0 ? args[0].getInt() : overrideUserId;
		if(userId >= numUsers) {
			error("Kinect2Receiver::handleSkeletonFeatures - userId too high " + userId);
			return;
		}

		String jointName = args[1].getString();
		Integer jointIndex = jointNameToIndex.get(jointName);
		if(jointIndex != null) {
			quat[0] = args[6].getFloat();	// quatx
			quat[1] = args[7].getFloat();	// quaty
			quat[2] = args[8].getFloat();	// quatz
			quat[3] = args[9].getFloat();	// quatw
			quatToEuler(quat);

			jointFeatures[0] = args[5].getFloat();	// confidence
			jointFeatures[1] = args[2].getFloat();	// posx
			jointFeatures[2] = args[3].getFloat();	// posy
			jointFeatures[3] = args[4].getFloat();	// posz
			jointFeatures[4] = quat[0];				// quatx
			jointFeatures[5] = quat[1];				// quaty
			jointFeatures[6] = quat[2];				// quatz
			jointFeatures[7] = quat[3];				// quatw
			jointFeatures[8] = (float)Math.toDegrees(euler[1]);	// yaw
			jointFeatures[9] = (float)Math.toDegrees(euler[2]);	// pitch
			jointFeatures[10] =(float)Math.toDegrees(euler[0]);	// roll

			usersMat.copyArrayToVectorPlanar(userId, 0, new int[] {0, jointIndex}, jointFeatures, numFeatures, 0);
		} else {
			error("Kinect2Receiver::handleSkeletonFeatures - unknown joint " + jointName);
		}
	}

	private float[] quatToEuler(float[] quat) {
		float x = quat[0];
		float y = quat[1];
		float z = quat[2];
		float w = quat[3];
        float roll, pitch, yaw;	// these are not mapped correct, fixed outside
        float test = x * y + z * w;
        if (test > 0.499) { // singularity at north pole
            pitch = 2 * (float)Math.atan2(x, w);
            yaw = 3.141592653f / 2;
            roll = 0;
            euler[0] = yaw;
            euler[1] = pitch;
            euler[2] = roll;
            return euler;
        }
        if (test < -0.499) { // singularity at south pole
            pitch = -2 * (float)Math.atan2(x, w);
            yaw = -3.141592653f / 2;
            roll = 0;
            euler[0] = yaw;
            euler[1] = pitch;
            euler[2] = roll;
            return euler;
        }
        float sqx = x * x;
        float sqy = y * y;
        float sqz = z * z;
        pitch = (float)Math.atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz);
        yaw = (float)Math.asin(2 * test);
        roll = (float)Math.atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz);
            euler[0] = yaw;
            euler[1] = pitch;
            euler[2] = roll;
        return euler;
	}

	protected void notifyDeleted() {
		post("Kinect2Receiver::notifyDeleted - goodbye");
		usersMat.freePeer();
	}

	public void dumpVersion() {
		post("Kinect2Receiver v" + version);
	}

}

/*
	// Help
	post("Hello World");	// print to console
	error("this is an error!"); // raise error
	outlet(0, "world!");	// send to outlet
	outletBang(0);			// send bang to outlet



*/












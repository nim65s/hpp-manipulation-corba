from hpp.corbaserver import Client
from hpp_manipulation import Client as ManipClient
from hpp_manipulation.robot import Robot

cl = Client()
mcl = ManipClient()

mcl.robot.create("test")
cl.robot.appendJoint("", "A/root_joint", "planar", [0, 0, 0, 0, 0, 0, 1])
cl.robot.createSphere("A/root_body", 0.001)
cl.robot.addObjectToJoint("A/root_joint", "A/root_body", [0, 0, 1, 0, 0, 0, 1])
mcl.robot.finishedRobot("A")
cl.robot.appendJoint("", "B/root_joint", "planar", [0, 0, 0, 0, 0, 0, 1])
cl.robot.createBox("B/root_body", 0.001, 0.002, 0.004)
cl.robot.addObjectToJoint("B/root_joint", "B/root_body", [0, 0, 1, 0, 0, 0, 1])
mcl.robot.finishedRobot("B")


robot = Robot()

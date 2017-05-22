using System;
using System.Threading;
using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Client.Services.UI;

namespace krpc
{
    internal class Program
    {
        private static void Main()
        {
            var conn = new Connection("Main thread");

            var canvas = conn.UI().StockCanvas;
            var screenSize = canvas.RectTransform.Size;
            var panel = canvas.AddPanel();

            var rect = panel.RectTransform;
            rect.Size = Tuple.Create(200d, 100d);
            rect.Position = Tuple.Create(110-(screenSize.Item1/2), 0d);

            var button = panel.AddButton("To the ORBIT");
            button.RectTransform.Position = Tuple.Create(0d, 20d);

            var buttonBack = panel.AddButton("Back");
            button.RectTransform.Position = Tuple.Create(0d, 80d);

            var text = panel.AddText("");
            text.RectTransform.Position = Tuple.Create(0d, -20d);
            text.Color = Tuple.Create(1d, 1d, 1d);
            text.Size = 18;

            var transferToOrbitThread = new Thread(TransferToOrbit);
            var getBackThread = new Thread(GetBack);

            while (true) 
            {
                if (button.Clicked)
                {

                    if (transferToOrbitThread.IsAlive)
                    {
                        transferToOrbitThread.Abort();
                    }
                    else
                    {
                        transferToOrbitThread = new Thread(TransferToOrbit);
                        transferToOrbitThread.Start();
                        button.Clicked = false;
                        button.Text.Content = "Activated";
                    }
                    
                }
                if (!buttonBack.Clicked) continue;

                if (getBackThread.IsAlive)
                {
                    getBackThread.Abort();
                }
                else
                {
                    getBackThread = new Thread(GetBack);
                    getBackThread.Start();
                    buttonBack.Clicked = false;
                    buttonBack.Text.Content = "Activated";
                }
            }
        }

        public static void TransferToOrbit()
        {
            var conn = new Connection("Launch into orbit");
            var vessel = conn.SpaceCenter().ActiveVessel;

            const int turnStartAltitude = 250;
            const float turnEndAltitude = 45000;
            const float targetAltitude = 150000;

            // Set up streams for telemetry
            var ut = conn.AddStream(() => conn.SpaceCenter().UT);
            var flight = vessel.Flight();
            var altitude = conn.AddStream(() => flight.MeanAltitude);
            var apoapsis = conn.AddStream(() => vessel.Orbit.ApoapsisAltitude);
            var stage4Resources = vessel.ResourcesInDecoupleStage(stage: 4, cumulative: false);
            var srbFuel = conn.AddStream(() => stage4Resources.Amount("SolidFuel"));

            // Pre-launch setup
            vessel.Control.SAS = false;
            vessel.Control.RCS = false;
            vessel.Control.Throttle = 1;

            // Countdown...
            Thread.Sleep(1000);
            Console.WriteLine("Launch!");

            // Activate the first stage
            vessel.Control.ActivateNextStage();
            vessel.AutoPilot.Engage();
            vessel.AutoPilot.TargetPitchAndHeading(90, 90);

            // Main ascent loop
            var srbsSeparated = false;
            double turnAngle = 0;
            while (true)
            {

                // Gravity turn
                if (altitude.Get() > turnStartAltitude && altitude.Get() < turnEndAltitude)
                {
                    double frac = (altitude.Get() - turnStartAltitude) / (turnEndAltitude - turnStartAltitude);
                    double newTurnAngle = frac * 90.0;
                    if (Math.Abs(newTurnAngle - turnAngle) > 0.5)
                    {
                        turnAngle = newTurnAngle;
                        vessel.AutoPilot.TargetPitchAndHeading((float)(90 - turnAngle), 90);
                    }
                }

                // Separate SRBs when finished
                if (!srbsSeparated)
                {
                    if (srbFuel.Get() < 0.1)
                    {
                        vessel.Control.ActivateNextStage();
                        srbsSeparated = true;
                        Console.WriteLine("SRBs separated");
                    }
                }

                // Decrease throttle when approaching target apoapsis
                if (apoapsis.Get() > targetAltitude * 0.9)
                {
                    Console.WriteLine("Approaching target apoapsis");
                    break;
                }
            }

            // Disable engines when target apoapsis is reached
            vessel.Control.Throttle = 0.25f;
            while (apoapsis.Get() < targetAltitude)
            {
            }
            Console.WriteLine("Target apoapsis reached");
            vessel.Control.Throttle = 0;

            // Wait until out of atmosphere
            Console.WriteLine("Coasting out of atmosphere");
            while (altitude.Get() < 70500)
            {
            }

            // Plan circularization burn (using vis-viva equation)
            Console.WriteLine("Planning circularization burn");
            double mu = vessel.Orbit.Body.GravitationalParameter;
            double r = vessel.Orbit.Apoapsis;
            double a1 = vessel.Orbit.SemiMajorAxis;
            double a2 = r;
            double v1 = Math.Sqrt(mu * ((2.0 / r) - (1.0 / a1)));
            double v2 = Math.Sqrt(mu * ((2.0 / r) - (1.0 / a2)));
            double deltaV = v2 - v1;
            var node = vessel.Control.AddNode(ut.Get() + vessel.Orbit.TimeToApoapsis, prograde: (float)deltaV);

            // Calculate burn time (using rocket equation)
            double f = vessel.AvailableThrust;
            double isp = vessel.SpecificImpulse * 9.82;
            double m0 = vessel.Mass;
            double m1 = m0 / Math.Exp(deltaV / isp);
            double flowRate = f / isp;
            double burnTime = (m0 - m1) / flowRate;

            // Orientate ship
            Console.WriteLine("Orientating ship for circularization burn");
            vessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
            vessel.AutoPilot.TargetDirection = Tuple.Create(0.0, 1.0, 0.0);
            vessel.AutoPilot.Wait();

            // Wait until burn
            Console.WriteLine("Waiting until circularization burn");
            var burnUt = ut.Get() + vessel.Orbit.TimeToApoapsis - (burnTime / 2.0);
            const double leadTime = 5;
            conn.SpaceCenter().WarpTo(burnUt - leadTime);

            // Execute burn
            Console.WriteLine("Ready to execute burn");
            var timeToApoapsis = conn.AddStream(() => vessel.Orbit.TimeToApoapsis);
            while (timeToApoapsis.Get() - (burnTime / 2.0) > 0)
            {
            }
            Console.WriteLine("Executing burn");
            vessel.Control.Throttle = 1;
            Thread.Sleep((int)((burnTime - 0.1) * 1000));
            Console.WriteLine("Fine tuning");
            vessel.Control.Throttle = 0.05f;
            var remainingBurn = conn.AddStream(() => node.RemainingBurnVector(node.ReferenceFrame));
            while (remainingBurn.Get().Item1 > 0)
            {
            }
            vessel.Control.Throttle = 0;
            node.Remove();

            Console.WriteLine("Launch complete");
        }

        public static void GetBack()
        {
            var conn = new Connection("Get Back");

           

            var vessel = conn.SpaceCenter().ActiveVessel;

            var flight = vessel.Flight();
            var altitude = conn.AddStream(() => flight.MeanAltitude);
            var speed = conn.AddStream(() => flight.VerticalSpeed);

            var retrograde = vessel.Flight(vessel.OrbitalReferenceFrame).Retrograde;

            
            conn.Drawing().AddDirection(retrograde, vessel.SurfaceVelocityReferenceFrame);

            vessel.AutoPilot.ReferenceFrame = vessel.OrbitalReferenceFrame;
            vessel.AutoPilot.TargetDirection = Tuple.Create(0d, -1d, 0d);
            vessel.AutoPilot.Engage();
            vessel.AutoPilot.Wait();
            vessel.Control.Throttle = 1f;

            while (altitude.Get()>=20000d || speed.Get()>=1)
            {

               

                vessel.AutoPilot.TargetDirection = Tuple.Create(0d, -1d, 0d);
                vessel.AutoPilot.Engage();
                vessel.AutoPilot.Wait();
            }
            
            vessel.AutoPilot.Disengage();

            while (true)
            {
                if (altitude.Get() <= 1500)
                {
                    foreach (var parachute in vessel.Parts.Parachutes)
                    {
                        parachute.Deploy();
                    }
                }
                return;
            }

            

          

           
        }
    }
}

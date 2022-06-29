using System;
using System.Threading;
using System.Threading.Tasks;

using ROS2;

namespace ConsoleApplication
{
    public static class RCLDotnetGuardCondition
    {
        static GuardCondition _guardCondition = null;

        public static void Main(string[] args)
        {
            RCLdotnet.Init();

            var node = RCLdotnet.CreateNode("guard_condition_example");

            _guardCondition = node.CreateGuardCondition(GuardConditionTriggered);

            Console.WriteLine(RCLdotnet.GetRMWIdentifier());
            Console.WriteLine($"ThreadId: {Environment.CurrentManagedThreadId}");

            _guardCondition.Trigger();

            RCLdotnet.Spin(node);
        }

        static void GuardConditionTriggered()
        {
            _ = TriggerAfterSomeTime();
        }

        static async Task TriggerAfterSomeTime()
        {
            // This is a multiple of the 500ms used in the loop in
            // RCLdotnet.Spin(). So the guard condition will be triggered nearly
            // at the same time as the wait times out.
            await Task.Delay(1000);

            // This will be called in an background worker thread as console
            // applications don't have an SynchronizationContext by default.
            _guardCondition.Trigger();
        }
    }
}

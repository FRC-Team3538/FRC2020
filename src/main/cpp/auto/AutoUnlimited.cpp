#include "auto/AutoUnlimited.hpp"

// Name for Smart Dash Chooser
std::string AutoUnlimited::GetName()
{
    return "1 - AutoUnlimited";
}

// Initialization
// Constructor requires a reference to the robot map
AutoUnlimited::AutoUnlimited(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoUnlimited::~AutoUnlimited() { }


//State Machine
void AutoUnlimited::NextState(){
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoUnlimited::Run()
{

        double fwd = 1.00;
        double rot = 0.00;
        IO.drivebase.Arcade(fwd, rot);
       


}
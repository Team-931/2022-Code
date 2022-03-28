/*This will contain the templates for functions to 
//  * automatically aim the ball 
    * automatically shoot the ball


 */


//Inclusion of other files to run the functions in this file

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Turret.h"
#include "subsystems/Ballevator.h"


class autoaim
: public frc2::CommandHelper<frc2::CommandBase, autoaim> //inherits from commandbase
{

public:

autoaim(Turret &t, Ballevator &b); //constructor for autoaim class

void Initialize() override;

void Execute() override; //execute function

private:

Turret &turret;
Ballevator &ballevator;


};
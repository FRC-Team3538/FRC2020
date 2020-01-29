#include "subsystem/RJVisionPipeline.hpp"

using namespace nt;

namespace vision
{

RJVisionPipeline::RJVisionPipeline()
{
}

void RJVisionPipeline::Init()
{
	table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	SetPipeline(0.0);
}

void RJVisionPipeline::Periodic()
{
	dx = table->GetNumber("tx", 0.0);
	dy = table->GetNumber("ty", 0.0);
	tv = table->GetNumber("tv", 0.0);
	pipe = table->GetNumber("pipeline", 0.0);
	pnpPoints = table->GetNumberArray("camtran", std::vector<double>());
}

RJVisionPipeline::visionData RJVisionPipeline::Run(int shotType) //Returns telemetry and changes pipeline
{
	RJVisionPipeline::visionData telemetry;
	int shotTypeCor = shotType;

	if (pnpPoints.size() >= 5)
	{
		pnpDist = sqrt(pow(pnpPoints[0], 2) + pow((pnpPoints[2] - 29), 2));
	}

	if (!pipeSwitchOS)
	{
		if ((shotType == 1) && (DistEstimation() > 188.0))
		{
			SetPipeline(DeterminePipeline(0));
			shotTypeCor = 0;
		}
		else
		{
			SetPipeline(DeterminePipeline(shotType));
		}
		pipeSwitchOS = true;
		telemetry.angle = 420.0;
		telemetry.distance = -1.0;
		telemetry.filled = false;
	}
	else
	{
		if (tv == 1.0)
		{
			telemetry.angle = dx;
			telemetry.distance = (shotTypeCor == 0) ? DistEstimation() : pnpDist;
			telemetry.filled = true;
		}
		else
		{
			telemetry.angle = 420.0;
			telemetry.distance = -1.0;
			telemetry.filled = false;
		}
	}
	return telemetry;
}

void RJVisionPipeline::SetPipeline(double pipeline)
{
	table->PutNumber("pipeline", pipeline);
	table->PutNumber("camMode", 0.0);
}

void RJVisionPipeline::UpdateSmartDash()
{
	frc::SmartDashboard::PutNumber("dx", dx);
	frc::SmartDashboard::PutNumber("pnpDist", sqrt(pow(pnpPoints[0], 2) + pow((pnpPoints[2] - 29), 2)));
}

double RJVisionPipeline::DistEstimation()
{
	double dist = 44.0 / (tan((dy + cameraAngle) * (3.1415 / 180)));
	return dist;
}

double RJVisionPipeline::DeterminePipeline(int shotType)
{
	switch (shotType)
	{
	case 0:
	{
		if ((DistEstimation() < 320.0) || (DistEstimation() > 113.0))
		{
			return 4.0;
		}
		else
		{
			return 0.0;
		}

		break;
	}

	case 1:
	{
		if (DistEstimation() > 94.0)
		{
			return 2.0;
		}
		else
		{
			return 1.0;
		}
		break;
	}
	default:
	{
		std::cout << "AHHHHHHHHHHHHHHHHHHHHHHHHHHHH" << std::endl;
		return 0.0;
		//SOMETHING WENT VERY WRONG!!!
	}
	}
}
} // namespace vision
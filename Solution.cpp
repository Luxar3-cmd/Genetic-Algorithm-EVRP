#include "Solution.hpp"

string feasibility_reason_to_string(FeasibilityReason reason) {
  switch (reason) {
  case FeasibilityReason::Feasible:
    return "Feasible";
  case FeasibilityReason::CapacityExceeded:
    return "Capacity Exceeded";
  case FeasibilityReason::FleetExceeded:
    return "Fleet Exceeded";
  case FeasibilityReason::BatteryInsufficient:
    return "Battery Insufficient";
  case FeasibilityReason::UnreachableClient:
    return "Unreachable Client";
  case FeasibilityReason::Unknown:
  default:
    return "Unknown";
  }
}

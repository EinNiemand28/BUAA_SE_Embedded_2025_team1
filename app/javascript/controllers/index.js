// Import and register all your controllers from the importmap under controllers/*

import { application } from "controllers/application"

// Eager load all controllers defined in the import map under controllers/**/*_controller
import { eagerLoadControllersFrom } from "@hotwired/stimulus-loading"
eagerLoadControllersFrom("controllers", application)

// Lazy load controllers as they appear in the DOM (remember not to preload controllers in import map!)
// import { lazyLoadControllersFrom } from "@hotwired/stimulus-loading"
// lazyLoadControllersFrom("controllers", application)

// import SidebarController from "./sidebar_controller"
// import RobotController from "./robot_controller"
// import DashboardRobotStatusController from "./dashboard_robot_status_controller"

// application.register("sidebar", SidebarController)
// application.register("robot", RobotController)
// application.register("dashboard-robot-status", DashboardRobotStatusController) 
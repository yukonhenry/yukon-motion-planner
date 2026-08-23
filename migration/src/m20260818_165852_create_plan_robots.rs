use crate::m20260730_213446_create_plans::Plans;
use crate::m20260818_015346_create_robots::Robots;
use sea_orm_migration::prelude::*;
use sea_orm_migration::schema::{integer, json_binary, json_binary_null, pk_auto};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260818_165852_create_plan_robots"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    /// Which robots a plan is for, and where each one enters it.
    ///
    /// A junction rather than a `robot_id` on `plans`, because both directions are
    /// many: a plan is a scenario that several robots may run at once, and one robot
    /// is reused across every plan it is tested on. A robot reaches a grid only
    /// through here — the plan already names the grid, so a second path to it would
    /// be a second answer to "which world is this robot in".
    ///
    /// The endpoints live on the *assignment*, not on the robot and not on the plan.
    /// A robot is a set of capabilities and has no home cell; a plan's `meta` holds
    /// one pair of endpoints, which is exactly the thing that stops being sufficient
    /// the moment two robots share a plan. Keeping them here is what lets two robots
    /// start from different corners of the same world.
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .create_table(
                Table::create()
                    .table(PlanRobots::Table)
                    .if_not_exists()
                    .col(pk_auto(PlanRobots::Id))
                    .col(integer(PlanRobots::PlanId))
                    .col(integer(PlanRobots::RobotId))
                    // `[x, y]`, the same shape `plans.meta` already stores endpoints in,
                    // so the two agree without a conversion in between.
                    .col(json_binary(PlanRobots::StartVertex))
                    // Nullable because a robot may be along for the ride — patrolling, or
                    // acting as a moving obstacle for the others — with no goal of its own.
                    // Null is "no destination", which a default could not express.
                    .col(json_binary_null(PlanRobots::GoalVertex))
                    // Cascade on both sides: an assignment is meaningless without either
                    // end, and leaving one behind would strand a row pointing at nothing.
                    .foreign_key(
                        ForeignKey::create()
                            .name("fk-plan_robots-plan_id")
                            .from(PlanRobots::Table, PlanRobots::PlanId)
                            .to(Plans::Table, Plans::Id)
                            .on_update(ForeignKeyAction::Cascade)
                            .on_delete(ForeignKeyAction::Cascade),
                    )
                    .foreign_key(
                        ForeignKey::create()
                            .name("fk-plan_robots-robot_id")
                            .from(PlanRobots::Table, PlanRobots::RobotId)
                            .to(Robots::Table, Robots::Id)
                            .on_update(ForeignKeyAction::Cascade)
                            .on_delete(ForeignKeyAction::Cascade),
                    )
                    .to_owned(),
            )
            .await?;

        // One assignment per robot per plan. Two rows would be one robot claiming two
        // start cells in a single world, and the replanner would have no way to pick.
        manager
            .create_index(
                Index::create()
                    .if_not_exists()
                    .name("idx-plan_robots-plan_id-robot_id")
                    .table(PlanRobots::Table)
                    .col(PlanRobots::PlanId)
                    .col(PlanRobots::RobotId)
                    .unique()
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_table(Table::drop().table(PlanRobots::Table).if_exists().to_owned())
            .await
    }
}

#[derive(DeriveIden)]
pub enum PlanRobots {
    Table,
    Id,
    PlanId,
    RobotId,
    StartVertex,
    GoalVertex,
}

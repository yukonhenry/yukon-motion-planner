use crate::m20260818_015346_create_robots::Robots;
use crate::m20260901_171859_create_grid_world_states::GridWorldStates;
use sea_orm_migration::{prelude::*, schema::*};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260901_192414_create_route_plans"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .create_table(
                Table::create()
                    .table(RoutePlans::Table)
                    .if_not_exists()
                    .col(pk_auto(RoutePlans::Id))
                    .col(string(RoutePlans::Name).not_null())
                    .col(integer(RoutePlans::GridWorldStateId).not_null())
                    // Required: planning is robot-first. A route is computed *for* a
                    // machine — its speed and cadence are what a run is driven by — so a
                    // route with nobody to drive it is a plan for nothing, and would only be
                    // discovered as unrunnable at the moment someone pressed Run.
                    .col(integer(RoutePlans::RobotId).not_null())
                    // The problem, as `[x, y]` cell pairs. Columns rather than fields of
                    // `meta`, because every reader needs them — the replanner, the bounds
                    // check, the canvas markers — and none of them should be parsing a blob
                    // to find out where a route was meant to go.
                    //
                    // Both required: a route without a destination is not a route. A robot
                    // with somewhere to be but nowhere to go is an assignment, not a plan.
                    .col(json_binary(RoutePlans::SrcVertex).not_null())
                    .col(json_binary(RoutePlans::DestVertex).not_null())
                    // The answer: the cells the route runs through, empty when the goal is
                    // walled off.
                    .col(json_binary(RoutePlans::RouteVertices).not_null())
                    .col(json_binary(RoutePlans::Meta))
                    // Cascades one way only. A world outlives the routes planned in it —
                    // deleting a route is not a claim that the moment never happened, and
                    // sibling routes may still point at the same row — but a route whose
                    // world is gone describes nothing and goes with it.
                    .foreign_key(
                        ForeignKey::create()
                            .name("fk-route_plans-grid_world_state_id")
                            .from(RoutePlans::Table, RoutePlans::GridWorldStateId)
                            .to(GridWorldStates::Table, GridWorldStates::Id)
                            .on_update(ForeignKeyAction::Cascade)
                            .on_delete(ForeignKeyAction::Cascade),
                    )
                    .foreign_key(
                        ForeignKey::create()
                            .name("fk-route_plans-robot_id")
                            .from(RoutePlans::Table, RoutePlans::RobotId)
                            .to(Robots::Table, Robots::Id)
                            .on_update(ForeignKeyAction::Cascade)
                            .on_delete(ForeignKeyAction::Cascade),
                    )
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_table(Table::drop().table(RoutePlans::Table).to_owned())
            .await
    }
}

#[derive(Iden)]
pub enum RoutePlans {
    Table,
    Id,
    Name,
    GridWorldStateId,
    RobotId,
    SrcVertex,
    DestVertex,
    RouteVertices,
    Meta,
}

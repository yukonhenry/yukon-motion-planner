use sea_orm_migration::prelude::*;
use sea_orm_migration::schema::{json_binary, pk_auto, string};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260818_015346_create_robots"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    /// A robot is a *spec*: a name and a bag of capabilities, and nothing about where it is.
    ///
    /// Deliberately no `grid_world_id`. A grid row is a frozen snapshot that forks a new row at
    /// `version + 1` on every edit, so a foreign key here would pin a robot to one snapshot
    /// rather than to a world — move an obstacle and the whole fleet would have to be
    /// recreated against the new id, and dropping an old version would cascade the robots
    /// away with it. Capabilities are geometry-independent; which world a robot is in is a
    /// property of the scenario, which `route_plans` records.
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .create_table(
                Table::create()
                    .table(Robots::Table)
                    .if_not_exists()
                    .col(pk_auto(Robots::Id))
                    .col(string(Robots::Name))
                    // Free-form while the shape of a capability is still being worked out:
                    // footprint, speed and kinematics all land here before any of them has
                    // settled enough to earn a column.
                    .col(json_binary(Robots::Capabilities))
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_table(Table::drop().table(Robots::Table).if_exists().to_owned())
            .await
    }
}

#[derive(Iden)]
pub enum Robots {
    Table,
    Id,
    Name,
    Capabilities,
}

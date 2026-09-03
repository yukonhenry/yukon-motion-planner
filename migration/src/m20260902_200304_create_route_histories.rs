use crate::m20260818_015346_create_robots::Robots;
use sea_orm_migration::{prelude::*, schema::*};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260902_200304_create_route_histories"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager.create_table(
            Table::create()
                .table(RouteHistories::Table)
                .if_not_exists()
                .col(pk_auto(RouteHistories::Id))
                .col(integer(RouteHistories::RobotId).not_null())
                .col(json_binary(RouteHistories::SrcVertex).not_null())
                .col(json_binary(RouteHistories::DestVertex).not_null())
                .col(json_binary(RouteHistories::RouteVertices).not_null())
                .foreign_key(
                    ForeignKey::create()
                        .name("fk-route_histories-robot_id")
                        .from(RouteHistories::Table, RouteHistories::RobotId)
                        .to(Robots::Table, Robots::Id)
                        .on_update(ForeignKeyAction::Cascade)
                        .on_delete(ForeignKeyAction::Cascade),
                )
                .to_owned(),
        ).await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_table(Table::drop().table(RouteHistories::Table).to_owned())
            .await
    }
}

#[derive(Iden)]
enum RouteHistories {
    Table,
    Id,
    RobotId,
    SrcVertex,
    DestVertex,
    RouteVertices,
}

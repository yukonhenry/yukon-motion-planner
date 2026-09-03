use crate::m20260727_180611_create_grid_worlds::GridWorlds;
use sea_orm_migration::{prelude::*, schema::*};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260901_171859_create_grid_world_states"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .create_table(
                Table::create()
                    .table(GridWorldStates::Table)
                    .if_not_exists()
                    .col(pk_auto(GridWorldStates::Id))
                    .col(integer(GridWorldStates::GridWorldId).not_null())
                    .col(json_binary(GridWorldStates::ObsPolygons).not_null())
                    .col(timestamp_with_time_zone(GridWorldStates::Timestamp).not_null()
                        .default(Expr::current_timestamp()))
                    .col(integer(GridWorldStates::SequenceId).not_null().default(0))
                    .foreign_key(
                        ForeignKey::create()
                            .name("fk-grid_world_states-grid_world_id")
                            .from(GridWorldStates::Table, GridWorldStates::GridWorldId)
                            .to(GridWorlds::Table, GridWorlds::Id)
                            .on_update(ForeignKeyAction::Cascade)
                            .on_delete(ForeignKeyAction::Cascade),
                    )
                    .to_owned(),
            )
            .await?;

        manager
            .create_index(
                Index::create()
                    .name("idx-grid_world_states-grid_world_id-sequence_id")
                    .table(GridWorldStates::Table)
                    .col(GridWorldStates::GridWorldId)
                    .col(GridWorldStates::SequenceId)
                    .unique()
                    .to_owned(),
            )
            .await?;
        Ok(())
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_index(
                Index::drop()
                    .name("idx-grid_world_states-grid_world_id-sequence_id")
                    .table(GridWorldStates::Table)
                    .to_owned(),
            )
            .await?;
        manager
            .drop_table(Table::drop().table(GridWorldStates::Table).to_owned())
            .await?;
        Ok(())
    }
}

#[derive(Iden)]
pub enum GridWorldStates {
    Table,
    Id,
    GridWorldId,
    ObsPolygons,
    Timestamp,
    SequenceId,
}


use sea_orm_migration::{prelude::*, schema::*};

use crate::m20260727_180611_create_grid_worlds::GridWorlds;

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260730_213446_create_plans"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .create_table(
                Table::create()
                    .table(Plans::Table)
                    .if_not_exists()
                    .col(pk_auto(Plans::Id))
                    .col(integer(Plans::GridId))
                    .col(string(Plans::Name))
                    .col(json_binary(Plans::Vertices))
                    .col(json_binary(Plans::Meta))
                    .foreign_key(
                        ForeignKey::create()
                            .name("fk-plans-grid_id")
                            .from(Plans::Table, Plans::GridId)
                            .to(GridWorlds::Table, GridWorlds::Id)
                            .on_update(ForeignKeyAction::Cascade)
                            .on_delete(ForeignKeyAction::Cascade)
                    )
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .drop_table(Table::drop().table(Plans::Table).to_owned())
            .await
    }
}

#[derive(Iden)]
enum Plans {
    Table,
    Id,
    GridId,
    Name,
    Vertices,
    Meta,
}

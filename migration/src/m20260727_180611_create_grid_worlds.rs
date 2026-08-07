use sea_orm_migration::{prelude::*, schema::*};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260727_180611_create_grid_worlds"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        // Replace the sample below with your own migration scripts
        manager
            .create_table(
                Table::create()
                    .table(GridWorlds::Table)
                    .if_not_exists()
                    .col(pk_auto(GridWorlds::Id))
                    .col(string(GridWorlds::Name).not_null())
                    .col(integer(GridWorlds::Width).not_null())
                    .col(integer(GridWorlds::Height).not_null())
                    .col(json_binary(GridWorlds::ObsPolygons))
                    .col(integer(GridWorlds::Version).not_null().default(0))
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        // Replace the sample below with your own migration scripts
        manager
            .drop_table(Table::drop().table(GridWorlds::Table).to_owned())
            .await
    }
}

#[derive(DeriveIden)]
pub enum GridWorlds {
    Table,
    Id,
    Name,
    Width,
    Height,
    ObsPolygons,
    Version,
}

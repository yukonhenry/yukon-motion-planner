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
                    .col(double(GridWorlds::SimInterval).not_null().default(1.0))
                    .col(integer(GridWorlds::Version).not_null().default(0))
                    .to_owned(),
            )
            .await?;

        manager
            .create_index(
                Index::create()
                    .if_not_exists()
                    .name("idx-grid_worlds-name-version")
                    .table(GridWorlds::Table)
                    .col(GridWorlds::Name)
                    .col(GridWorlds::Version)
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
                    .name("idx-grid_worlds-name-version")
                    .table(GridWorlds::Table)
                    .to_owned(),
            )
            .await?;
        manager
            .drop_table(Table::drop().table(GridWorlds::Table).to_owned())
            .await?;
        Ok(())
    }
}

#[derive(DeriveIden)]
pub enum GridWorlds {
    Table,
    Id,
    Name,
    Width,
    Height,
    Version,
    SimInterval,
}

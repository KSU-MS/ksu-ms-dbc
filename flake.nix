# I hate nix
{
  description = "A very basic flake that generates the DBCs by running a python script";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs =
    {
      flake-utils,
      self,
      nixpkgs,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkg_overlay = final: prev: {
          can_pkg = final.callPackage ./default.nix { };
        };
        custom_overlays = [ pkg_overlay ];

        pkgs = import nixpkgs {
          inherit system;
          overlays = [ pkg_overlay ];
        };

      in
      {
        overlays.default = nixpkgs.lib.composeManyExtensions custom_overlays;

        packages = {
          default = pkgs.can_pkg;
          can_pkg = pkgs.can_pkg;
        };
      }
    );
}

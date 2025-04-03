[![Open in Dev Container](https://img.shields.io/static/v1?label=Dev%20Containers&message=Open&color=blue&logo=visualstudiocode)](https://vscode.dev/redirect?url=vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://github.com/aica-technology/component-template)

# AICA Package Template

A software development kit for creating custom components for AICA applications.

Full documentation for creating, testing and packaging custom components is available at
https://docs.aica.tech/docs/category/custom-components.

You will need to have Docker installed in order to develop or build your component package
(see [Docker installation](https://docs.docker.com/get-docker/)).

## Create a custom package repository

To create a component package, create a new repository in GitHub using this repository as a template.

![Use this template menu](docs/creation-1.png)
![Create new repository menu](docs/creation-2.png)

Alternatively, you can also clone this repository locally and create a new repository from it.

```bash
git clone git@github.com:aica-technology/component-template.git my_component_package
```

## Rename the template component package 

This template repository uses the placeholder package name `template_component_package`. The first thing you should
do is rename the package to a meaningful custom name. This can be accomplished using the included bash script
[`rename_package.sh`](./rename_package.sh) (use `./rename_package.sh --help` for more information). For example:

```bash
./rename_package.sh my_custom_package [--dry-run]
```

If you want to include a second ROS package within the AICA package, you will need to create a new package folder in `source`
and add it in your `aica-package.toml` file under `[build.packages.name_of_new_package]` accordingly.

## Configure the package development environment

This template uses a devContainer configuration ([`.devcontainer.json`](./.devcontainer.json)) with base AICA Docker images
for a seamless integrated development experience.

Using VSCode and the Dev Containers extension, after creating and renaming the template package, simply open the repository
in a devcontainer using the "Reopen in Container" command.

Other IDEs such as JetBrains can similarly be configured to use development containers.

If any changes are made to aica-package.toml (including any package names or dependencies), remember to rebuild the devcontainer.

## Building your component package

You can build your package using the following command:

```bash
docker build -f aica-package.toml .
```

We use a custom Docker frontend instead of a Dockerfile, so all configuration of the build is stored in `aica-package.toml`.
As we are using `docker build` to build you can pass any Docker argument, like `-t <image_name>` to tag the image or `--platform <platform>` to build for a specific platform. Note that you can only build one platform at a time at the moment.

## Testing your component package

You can invoke any unit tests in your package by changing the docker build stage to `test`, e.g.:

```bash
docker build -f aica-package.toml --target test .
```

## Package configuration with `aica-package.toml`

All build configurations, metadata and dependencies are defined in [aica-package.toml](./aica-package.toml). Refer to the
[docs](https://docs.aica.tech/docs/reference/custom-components/aica-package-toml) for more details about the syntax.

### CLI configuration

The `aica-package.toml` file contains the configuration of the build process but you can override it using the CLI.

In order to do so, you will need to pass `--build-arg config.<key>=<value>` to the `docker build` command.

Example:

```bash
docker build -f aica-package.toml --build-arg config.build.cmake_args.SOME_FLAG=Release .
```

### Installing external dependencies

As the build is done in a Docker container, you will need to install external dependencies through `aica-package.toml`.

#### System libraries

You can add system libraries by adding the list of packages to install through `apt`:

```toml
[build.packages.component.dependencies.apt]
libyaml-cpp-dev = "*"
```

Note that the `*` is currently ignored but might be used in the future to specify a version.

#### Python packages

`aica-package.toml` will automatically install any Python packages specified in a `requirements.txt` file stored in your component package folder. However, you can change the name of that file or specify the packages to install directly in `aica-package.toml`:

```toml
[build.packages.component.dependencies.pip]
file = "requirements.txt"
# OR
[build.packages.component.dependencies.pip.packages]
numpy = "1.0.0"
```

## Using your component package

After you have built and tagged your package as a docker image, you can use it in your application. See the
[AICA documentation](https://docs.aica.tech/docs/getting-started/installation-and-launch#configuring-the-aica-system-image) for more details.

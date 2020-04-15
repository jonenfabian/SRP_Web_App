from django.urls import path
from . import views

urlpatterns=[
    path("",views.WelcomeView.as_view(),name='welcome'),
    path("create_srp_stp1/",views.upload_file_view,name='create_srp_stp1'),
    path("create_srp_stp2/",views.upload_db_view,name='create_srp_stp2'),
    path("cluster_success/",views.create_srp_cluster,name='cluster_success'),
    path("cluster_fail/",views.create_srp_cluster,name='cluster_fail'),
    path("create_srp_stp3/",views.Step3View.as_view(),name='create_srp_stp3'),
    path("cluster_convert/",views.cluster_convert,name='cluster_convert'),
    path("cluster_connect/",views.cluster_connect,name='cluster_connect'),
    path("review_srps/",views.show_srps,name='review_srps'),
    path('srpclusterdb/<int:pk>', views.SRPDetailView.as_view(), name='srpclusterdb_detail'),
    path('about_srp/',views.AboutSrpView.as_view(),name='about_srp')

]

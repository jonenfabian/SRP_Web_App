from django.shortcuts import render,redirect
from django.template import Context, Template
from .models import *
from .forms import *
from django.views.generic import (TemplateView,ListView,
                                  DetailView,CreateView,
                                  UpdateView,DeleteView)
import sys
sys.path.append('..')
import static.Synthetic_Robot_Program.SRP_Cluster_Generator as generate_srp_cluster
import static.Synthetic_Robot_Program.SRP_Cluster_2_UR_Script_v2_beta as convert_srp_cluster

# Create your views here.
class WelcomeView(TemplateView):
    template_name = 'SRP_APP/welcome.html'

class CreateNewSRPView(TemplateView):
    template_name = 'SRP_APP/create_srp_stp1.html'
class Step3View(TemplateView):
    template_name = 'SRP_APP/create_srp_stp3.html'
class SRPDetailView(DetailView):
    model = SrpClusterDb
class AboutSrpView(TemplateView):
    template_name= 'SRP_APP/about_srp.html'
    
def upload_file_view(request):

    if request.method == 'POST':
        form = FileForm(request.POST, request.FILES)
        if form.is_valid():
            form.save()
            return redirect('create_srp_stp2')
    else:
        form = FileForm()
    return render(request, 'SRP_APP/create_srp_stp1.html', {'form' : form})

def upload_db_view(request):
    if request.method == 'POST':
        form = DBForm(request.POST, request.FILES)
        #if form.is_valid():
        form.save()
        return redirect('cluster_success') #TODO CHANGE THIS TO THE PRCESSING PAGE "create_srp_cluster"
    else:
        form = DBForm()
    return render(request, 'SRP_APP/create_srp_stp2.html', {'form' : form})

def create_srp_cluster(request):
    #import static.Synthetic_Robot_Program.SRP_Cluster_Generator as generate_srp_cluster
    #SRP_MAIN\static\Synthetic_Robot_Program\SRP_Cluster_Generator.py
    fitting_instruction={
		"procedural_step_0": {
			"original": "Assemble all BUSH_BEARING onto HPC_FRONT_BOTTOM.",
			"operation": "Assemble",
			"target": "NPN25984_C__CASING_HPC_FRONT_BOTTOM.asm;0;2:",
			"relation": "FW51528_D__BUSH_BEARING;0;92:"
		},
		# "procedural_step_1": {
		# 	"original": "Assemble all BUSH_BEARING onto HPC_FRONT_BOTTOM.",
		# 	"operation": "Assemble",
		# 	"target": "NPN25984_C__CASING_HPC_FRONT_BOTTOM.asm;0;2:",
		# 	"relation": "FW51530_D__BUSH_BEARING;0;92:"
		# }
	       }
    # try:

    results,cluster=generate_srp_cluster.srp_cluster_generator(fitting_instruction) #Creates a data cluster as a pickle file
    cluster=Context(cluster)
    results=Context(results)

    return render(request, 'SRP_APP/cluster_success.html', {'inject_used_fi' : {'results':results,"cluster":cluster,"filter_list":["original","operation","target","relation","operation_skill"]}}) #TODO must be created
    #except:
        #print("Sorry that didnt work")
        #return render(request, 'SRP_APP/cluster_fail.html', {'inject_used_fi' : fitting_instruction})

def cluster_convert(request):
    script=convert_srp_cluster.convert_cluster_2_script()
    script_file = open("./static/generated_script/SRP.script", 'w')
    script_file.write(script)
    script_file.close
    return render(request, 'SRP_APP/cluster_convert.html', {'ur_script' : script})


def cluster_connect(request):
    script="Test"
    return render(request, 'SRP_APP/welcome.html', {'ur_script' : script})

##View SRP SECTION
#def show_srps(request)
def show_srps(request):
    review_srps = SrpClusterDb.objects.all()
    return render(request,'SRP_APP/review_srps.html',{"review_srps":review_srps})
